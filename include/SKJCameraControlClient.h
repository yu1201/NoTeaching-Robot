#pragma once

#include <QString>

class QLibrary;

struct SKJCameraParameterValues
{
    int exposure = 0;
    int gain = 0;
    int binarize = 0;
};

class SKJCameraControlClient
{
public:
    enum class Parameter
    {
        Exposure,
        Gain,
        Binarize
    };

    SKJCameraControlClient();
    ~SKJCameraControlClient();

    SKJCameraControlClient(const SKJCameraControlClient&) = delete;
    SKJCameraControlClient& operator=(const SKJCameraControlClient&) = delete;

    bool EnsureConnected(const QString& cameraIP, int cameraPort, QString* error = nullptr);
    bool ReadParameters(SKJCameraParameterValues& values, QString* error = nullptr);
    bool SetParameter(Parameter parameter, int value, QString* error = nullptr);
    bool SetLaserEnabled(bool enabled, QString* error = nullptr);
    void Disconnect();

    bool IsConnected() const;
    QString CurrentTargetText() const;
    QString SdkVersion() const;
    QString DllPath() const;

private:
    using CameraHandle = void*;

    using CreateFn = CameraHandle(__cdecl*)();
    using DestroyFn = void(__cdecl*)(CameraHandle);
    using ConnectFn = int(__cdecl*)(CameraHandle, const char*, int);
    using DisconnectFn = void(__cdecl*)(CameraHandle);
    using IsConnectedFn = int(__cdecl*)(CameraHandle);
    using SetTimeoutFn = int(__cdecl*)(CameraHandle, int);
    using GetIntFn = int(__cdecl*)(CameraHandle, int*);
    using SetIntFn = int(__cdecl*)(CameraHandle, int);
    using CommandFn = int(__cdecl*)(CameraHandle);
    using VersionFn = const char*(__cdecl*)();
    using ErrorStringFn = const char*(__cdecl*)(int);

    bool LoadSdk(QString* error);
    bool ResolveSymbol(const char* name, void** target, QString* error);
    QString ErrorText(const QString& action, int code) const;
    void DestroyHandle();

    QLibrary* m_library = nullptr;
    CameraHandle m_handle = nullptr;
    QString m_currentIP;
    int m_currentPort = 0;

    CreateFn m_create = nullptr;
    DestroyFn m_destroy = nullptr;
    ConnectFn m_connect = nullptr;
    DisconnectFn m_disconnect = nullptr;
    IsConnectedFn m_isConnected = nullptr;
    SetTimeoutFn m_setConnectTimeout = nullptr;
    SetTimeoutFn m_setCommandTimeout = nullptr;
    GetIntFn m_getExposure = nullptr;
    SetIntFn m_setExposure = nullptr;
    GetIntFn m_getGain = nullptr;
    SetIntFn m_setGain = nullptr;
    GetIntFn m_getBinarize = nullptr;
    SetIntFn m_setBinarize = nullptr;
    CommandFn m_laserOn = nullptr;
    CommandFn m_laserOff = nullptr;
    VersionFn m_getVersion = nullptr;
    ErrorStringFn m_getErrorString = nullptr;
};
