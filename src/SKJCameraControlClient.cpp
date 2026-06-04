#include "SKJCameraControlClient.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QLibrary>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr int SKJ_OK = 0;
constexpr int kConnectTimeoutMs = 1500;
constexpr int kCommandTimeoutMs = 3000;
constexpr int kExposureUnitScale = 25;

int DeviceExposureToDisplay(int deviceValue)
{
    if (deviceValue <= 0)
    {
        return 0;
    }
    if (deviceValue > std::numeric_limits<int>::max() / kExposureUnitScale)
    {
        return std::numeric_limits<int>::max();
    }
    return deviceValue * kExposureUnitScale;
}

int DisplayExposureToDevice(int displayValue)
{
    return std::max(1, static_cast<int>(std::lround(static_cast<double>(displayValue) / kExposureUnitScale)));
}

QString FindProjectFilePath(const QString& relativePath)
{
    const QString normalized = QDir::fromNativeSeparators(relativePath);
    const QStringList roots = {
        QCoreApplication::applicationDirPath(),
        QDir::currentPath()
    };

    for (const QString& root : roots)
    {
        QDir dir(root);
        for (int depth = 0; depth < 6; ++depth)
        {
            const QString candidate = dir.filePath(normalized);
            if (QFileInfo::exists(candidate))
            {
                return QFileInfo(candidate).absoluteFilePath();
            }
            if (!dir.cdUp())
            {
                break;
            }
        }
    }

    return QDir(QDir::currentPath()).filePath(normalized);
}
}

SKJCameraControlClient::SKJCameraControlClient()
{
}

SKJCameraControlClient::~SKJCameraControlClient()
{
    Disconnect();
    if (m_library != nullptr)
    {
        m_library->unload();
        delete m_library;
        m_library = nullptr;
    }
}

bool SKJCameraControlClient::EnsureConnected(const QString& cameraIP, int cameraPort, QString* error)
{
    const QString trimmedIP = cameraIP.trimmed();
    if (trimmedIP.isEmpty())
    {
        if (error != nullptr)
        {
            *error = "相机 IP 为空，无法连接参数控制。";
        }
        return false;
    }
    if (cameraPort <= 0)
    {
        if (error != nullptr)
        {
            *error = "相机端口无效，无法连接参数控制。";
        }
        return false;
    }
    if (!LoadSdk(error))
    {
        return false;
    }

    if (m_handle != nullptr
        && m_currentIP == trimmedIP
        && m_currentPort == cameraPort
        && m_isConnected != nullptr
        && m_isConnected(m_handle) == SKJ_OK)
    {
        return true;
    }

    DestroyHandle();
    m_handle = m_create != nullptr ? m_create() : nullptr;
    if (m_handle == nullptr)
    {
        if (error != nullptr)
        {
            *error = "创建 SKJCamera 控制句柄失败。";
        }
        return false;
    }

    if (m_setConnectTimeout != nullptr)
    {
        m_setConnectTimeout(m_handle, kConnectTimeoutMs);
    }
    if (m_setCommandTimeout != nullptr)
    {
        m_setCommandTimeout(m_handle, kCommandTimeoutMs);
    }

    const QByteArray ipBytes = trimmedIP.toLocal8Bit();
    const int ret = m_connect(m_handle, ipBytes.constData(), cameraPort);
    if (ret != SKJ_OK)
    {
        if (error != nullptr)
        {
            *error = ErrorText(QString("连接相机参数控制 %1:%2").arg(trimmedIP).arg(cameraPort), ret);
        }
        DestroyHandle();
        return false;
    }

    m_currentIP = trimmedIP;
    m_currentPort = cameraPort;
    return true;
}

bool SKJCameraControlClient::ReadParameters(SKJCameraParameterValues& values, QString* error)
{
    if (!IsConnected())
    {
        if (error != nullptr)
        {
            *error = "SKJCamera 参数控制未连接。";
        }
        return false;
    }

    int exposure = 0;
    int gain = 0;
    int binarize = 0;
    int ret = m_getExposure(m_handle, &exposure);
    if (ret != SKJ_OK)
    {
        if (error != nullptr)
        {
            *error = ErrorText("读取曝光", ret);
        }
        return false;
    }
    ret = m_getGain(m_handle, &gain);
    if (ret != SKJ_OK)
    {
        if (error != nullptr)
        {
            *error = ErrorText("读取增益", ret);
        }
        return false;
    }
    ret = m_getBinarize(m_handle, &binarize);
    if (ret != SKJ_OK)
    {
        if (error != nullptr)
        {
            *error = ErrorText("读取二值化阈值", ret);
        }
        return false;
    }

    values.exposure = DeviceExposureToDisplay(exposure);
    values.gain = gain;
    values.binarize = binarize;
    return true;
}

bool SKJCameraControlClient::SetParameter(Parameter parameter, int value, QString* error)
{
    if (!IsConnected())
    {
        if (error != nullptr)
        {
            *error = "SKJCamera 参数控制未连接。";
        }
        return false;
    }

    int ret = SKJ_OK;
    QString action;
    switch (parameter)
    {
    case Parameter::Exposure:
        action = "设置曝光";
        ret = m_setExposure(m_handle, DisplayExposureToDevice(value));
        break;
    case Parameter::Gain:
        action = "设置增益";
        ret = m_setGain(m_handle, value);
        break;
    case Parameter::Binarize:
        action = "设置二值化阈值";
        ret = m_setBinarize(m_handle, value);
        break;
    }

    if (ret != SKJ_OK)
    {
        if (error != nullptr)
        {
            *error = ErrorText(action, ret);
        }
        return false;
    }
    return true;
}

bool SKJCameraControlClient::SetLaserEnabled(bool enabled, QString* error)
{
    if (!IsConnected())
    {
        if (error != nullptr)
        {
            *error = "SKJCamera 参数控制未连接。";
        }
        return false;
    }

    const QString action = enabled ? QStringLiteral("打开激光") : QStringLiteral("关闭激光");
    const int ret = enabled ? m_laserOn(m_handle) : m_laserOff(m_handle);
    if (ret != SKJ_OK)
    {
        if (error != nullptr)
        {
            *error = ErrorText(action, ret);
        }
        return false;
    }
    return true;
}

void SKJCameraControlClient::Disconnect()
{
    DestroyHandle();
    m_currentIP.clear();
    m_currentPort = 0;
}

bool SKJCameraControlClient::IsConnected() const
{
    return m_handle != nullptr
        && m_isConnected != nullptr
        && m_isConnected(m_handle) == SKJ_OK;
}

QString SKJCameraControlClient::CurrentTargetText() const
{
    if (m_currentIP.isEmpty() || m_currentPort <= 0)
    {
        return QString();
    }
    return QString("%1:%2").arg(m_currentIP).arg(m_currentPort);
}

QString SKJCameraControlClient::SdkVersion() const
{
    if (m_getVersion == nullptr)
    {
        return QString();
    }
    const char* version = m_getVersion();
    return version != nullptr ? QString::fromUtf8(version) : QString();
}

QString SKJCameraControlClient::DllPath() const
{
    const QString localDll = QDir(QCoreApplication::applicationDirPath()).filePath("SKJCamera.dll");
    if (QFileInfo::exists(localDll))
    {
        return QFileInfo(localDll).absoluteFilePath();
    }
    return FindProjectFilePath("SDK/SKJCamera/bin/x64/Release/SKJCamera.dll");
}

bool SKJCameraControlClient::LoadSdk(QString* error)
{
    if (m_library != nullptr && m_library->isLoaded())
    {
        return true;
    }

    if (m_library == nullptr)
    {
        m_library = new QLibrary(DllPath());
    }
    if (!m_library->load())
    {
        if (error != nullptr)
        {
            *error = QString("加载 SKJCamera.dll 失败：%1\n路径：%2")
                .arg(m_library->errorString(), DllPath());
        }
        return false;
    }

    return ResolveSymbol("SKJCamera_Create", reinterpret_cast<void**>(&m_create), error)
        && ResolveSymbol("SKJCamera_Destroy", reinterpret_cast<void**>(&m_destroy), error)
        && ResolveSymbol("SKJCamera_Connect", reinterpret_cast<void**>(&m_connect), error)
        && ResolveSymbol("SKJCamera_Disconnect", reinterpret_cast<void**>(&m_disconnect), error)
        && ResolveSymbol("SKJCamera_IsConnected", reinterpret_cast<void**>(&m_isConnected), error)
        && ResolveSymbol("SKJCamera_SetConnectTimeout", reinterpret_cast<void**>(&m_setConnectTimeout), error)
        && ResolveSymbol("SKJCamera_SetCommandTimeout", reinterpret_cast<void**>(&m_setCommandTimeout), error)
        && ResolveSymbol("SKJCamera_GetExposure", reinterpret_cast<void**>(&m_getExposure), error)
        && ResolveSymbol("SKJCamera_SetExposure", reinterpret_cast<void**>(&m_setExposure), error)
        && ResolveSymbol("SKJCamera_GetGain", reinterpret_cast<void**>(&m_getGain), error)
        && ResolveSymbol("SKJCamera_SetGain", reinterpret_cast<void**>(&m_setGain), error)
        && ResolveSymbol("SKJCamera_GetBinarize", reinterpret_cast<void**>(&m_getBinarize), error)
        && ResolveSymbol("SKJCamera_SetBinarize", reinterpret_cast<void**>(&m_setBinarize), error)
        && ResolveSymbol("SKJCamera_LaserOn", reinterpret_cast<void**>(&m_laserOn), error)
        && ResolveSymbol("SKJCamera_LaserOff", reinterpret_cast<void**>(&m_laserOff), error)
        && ResolveSymbol("SKJCamera_GetVersion", reinterpret_cast<void**>(&m_getVersion), error)
        && ResolveSymbol("SKJCamera_GetErrorString", reinterpret_cast<void**>(&m_getErrorString), error);
}

bool SKJCameraControlClient::ResolveSymbol(const char* name, void** target, QString* error)
{
    if (target == nullptr)
    {
        return false;
    }
    *target = reinterpret_cast<void*>(m_library->resolve(name));
    if (*target == nullptr)
    {
        if (error != nullptr)
        {
            *error = QString("SKJCamera.dll 缺少导出函数：%1").arg(QString::fromLatin1(name));
        }
        return false;
    }
    return true;
}

QString SKJCameraControlClient::ErrorText(const QString& action, int code) const
{
    const char* text = m_getErrorString != nullptr ? m_getErrorString(code) : nullptr;
    return QString("%1失败：%2 (%3)")
        .arg(action)
        .arg(code)
        .arg(text != nullptr ? QString::fromUtf8(text) : QStringLiteral("unknown"));
}

void SKJCameraControlClient::DestroyHandle()
{
    if (m_handle == nullptr)
    {
        return;
    }
    if (m_disconnect != nullptr)
    {
        m_disconnect(m_handle);
    }
    if (m_destroy != nullptr)
    {
        m_destroy(m_handle);
    }
    m_handle = nullptr;
}
