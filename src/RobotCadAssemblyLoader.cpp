#include "RobotCadAssemblyLoader.h"

#include "OpenCascadeOperationGuard.h"

#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>
#include <XCAFPrs_DocumentNode.hxx>

#if OCC_VERSION_MAJOR != 7 || OCC_VERSION_MINOR != 9 || OCC_VERSION_MAINTENANCE != 3
#error "RobotCadAssemblyLoader requires Open CASCADE Technology 7.9.3."
#endif

#include <QByteArray>
#include <QCryptographicHash>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>
#include <QRegularExpression>

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <new>
#include <utility>
#include <vector>

namespace
{
constexpr qint64 kHardMaximumFileBytes = 512LL * 1024LL * 1024LL;

struct DocumentGuard
{
    Handle(TDocStd_Document) document;

    ~DocumentGuard() noexcept
    {
        if (document.IsNull()) return;
        try
        {
            XCAFApp_Application::GetApplication()->Close(document);
        }
        catch (...)
        {
            // 清理失败不能覆盖原始读取结果或异常。
        }
    }
};

struct SelectedComponent
{
    TopoDS_Shape shape;
    qsizetype statisticsIndex = -1;
};

class ScopedStepCodePage
{
public:
    explicit ScopedStepCodePage(const char* codePage)
    {
        const Standard_CString previous = Interface_Static::CVal("read.step.codepage");
        if (previous != nullptr) m_previous = QByteArray(previous);
        m_wasSet = Interface_Static::SetCVal("read.step.codepage", codePage) != Standard_False;
    }

    ~ScopedStepCodePage() noexcept
    {
        if (!m_wasSet || m_previous.isEmpty()) return;
        try
        {
            Interface_Static::SetCVal("read.step.codepage", m_previous.constData());
        }
        catch (...)
        {
        }
    }

    bool IsValid() const { return m_wasSet; }

private:
    QByteArray m_previous;
    bool m_wasSet = false;
};

bool IsValidUtf8(std::istream& input)
{
    std::array<char, 64 * 1024> buffer{};
    int continuationBytes = 0;
    std::uint8_t nextMinimum = 0x80;
    std::uint8_t nextMaximum = 0xBF;
    while (input.good())
    {
        input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const std::streamsize count = input.gcount();
        for (std::streamsize index = 0; index < count; ++index)
        {
            const auto byte = static_cast<std::uint8_t>(
                static_cast<unsigned char>(buffer[static_cast<size_t>(index)]));
            if (continuationBytes > 0)
            {
                if (byte < nextMinimum || byte > nextMaximum) return false;
                --continuationBytes;
                nextMinimum = 0x80;
                nextMaximum = 0xBF;
                continue;
            }
            if (byte <= 0x7F) continue;
            if (byte >= 0xC2 && byte <= 0xDF)
            {
                continuationBytes = 1;
            }
            else if (byte == 0xE0)
            {
                continuationBytes = 2;
                nextMinimum = 0xA0;
            }
            else if ((byte >= 0xE1 && byte <= 0xEC) || (byte >= 0xEE && byte <= 0xEF))
            {
                continuationBytes = 2;
            }
            else if (byte == 0xED)
            {
                continuationBytes = 2;
                nextMaximum = 0x9F;
            }
            else if (byte == 0xF0)
            {
                continuationBytes = 3;
                nextMinimum = 0x90;
            }
            else if (byte >= 0xF1 && byte <= 0xF3)
            {
                continuationBytes = 3;
            }
            else if (byte == 0xF4)
            {
                continuationBytes = 3;
                nextMaximum = 0x8F;
            }
            else
            {
                return false;
            }
        }
    }
    return continuationBytes == 0 && !input.bad();
}

QString ReadFailureText(IFSelect_ReturnStatus status)
{
    switch (status)
    {
    case IFSelect_RetVoid:
        return QStringLiteral("STEP 文件中没有可读取的数据。");
    case IFSelect_RetError:
        return QStringLiteral("STEP 文件格式或输入参数错误。");
    case IFSelect_RetFail:
        return QStringLiteral("Open CASCADE 解析 STEP 文件失败。");
    case IFSelect_RetStop:
        return QStringLiteral("STEP 文件解析被底层读取器中止。");
    case IFSelect_RetDone:
        break;
    }
    return QStringLiteral("STEP 文件返回了未知的读取状态。");
}

QString ExceptionDetail(const char* message)
{
    if (message == nullptr || *message == '\0') return QString();
    QString text = QString::fromLocal8Bit(message).simplified();
    constexpr qsizetype kMaximumDetailCharacters = 400;
    if (text.size() > kMaximumDetailCharacters)
        text = text.left(kMaximumDetailCharacters) + QStringLiteral("...");
    return text;
}

QString LabelName(const TDF_Label& label)
{
    if (label.IsNull()) return QString();
    Handle(TDataStd_Name) attribute;
    if (!label.FindAttribute(TDataStd_Name::GetID(), attribute) || attribute.IsNull())
        return QString();
#ifdef _WIN32
    return QString::fromWCharArray(attribute->Get().ToWideString()).trimmed();
#else
    const TCollection_ExtendedString& value = attribute->Get();
    QByteArray utf8(value.LengthOfCString() + 1, '\0');
    Standard_PCharacter destination = utf8.data();
    value.ToUTF8CString(destination);
    return QString::fromUtf8(utf8.constData()).trimmed();
#endif
}

QStringList ReadLengthUnitNames(STEPControl_Reader& reader)
{
    TColStd_SequenceOfAsciiString lengthNames;
    TColStd_SequenceOfAsciiString angleNames;
    TColStd_SequenceOfAsciiString solidAngleNames;
    reader.FileUnits(lengthNames, angleNames, solidAngleNames);

    QStringList result;
    for (Standard_Integer index = 1; index <= lengthNames.Length(); ++index)
    {
        const QString name = QString::fromLatin1(lengthNames.Value(index).ToCString()).trimmed();
        if (!name.isEmpty() && !result.contains(name, Qt::CaseInsensitive)) result.append(name);
    }
    return result;
}

bool ShapeBounds(const TopoDS_Shape& shape, RobotCadAssemblyLoader::Bounds& bounds)
{
    bounds = RobotCadAssemblyLoader::Bounds();
    if (shape.IsNull()) return false;

    Bnd_Box box;
    BRepBndLib::Add(shape, box, Standard_False);
    if (box.IsVoid() || box.IsOpen()) return false;

    Standard_Real xMinimum = 0.0;
    Standard_Real yMinimum = 0.0;
    Standard_Real zMinimum = 0.0;
    Standard_Real xMaximum = 0.0;
    Standard_Real yMaximum = 0.0;
    Standard_Real zMaximum = 0.0;
    box.Get(xMinimum, yMinimum, zMinimum, xMaximum, yMaximum, zMaximum);
    bounds.minimumMm = Eigen::Vector3d(xMinimum, yMinimum, zMinimum);
    bounds.maximumMm = Eigen::Vector3d(xMaximum, yMaximum, zMaximum);
    bounds.valid = bounds.minimumMm.allFinite() && bounds.maximumMm.allFinite()
        && (bounds.maximumMm.array() >= bounds.minimumMm.array()).all();
    return bounds.valid;
}

qsizetype ShapeCount(const TopoDS_Shape& shape, TopAbs_ShapeEnum shapeType)
{
    if (shape.IsNull()) return 0;
    TopTools_IndexedMapOfShape shapes;
    TopExp::MapShapes(shape, shapeType, shapes);
    return static_cast<qsizetype>(shapes.Extent());
}

int JointIndexFromName(const QString& name)
{
    static const QRegularExpression pattern(
        QStringLiteral(R"(J\s*([0-6])\s*$)"),
        QRegularExpression::CaseInsensitiveOption);
    const QRegularExpressionMatch match = pattern.match(name.trimmed());
    if (!match.hasMatch()) return -1;
    return match.captured(1).toInt();
}

int JointIndexFromNames(const QString& productName, const QString& instanceName)
{
    const int productIndex = JointIndexFromName(productName);
    return productIndex >= 0 ? productIndex : JointIndexFromName(instanceName);
}

bool IsPipelineName(const QString& name)
{
    const QString simplified = name.simplified();
    return simplified.contains(QStringLiteral("管线"))
        || simplified.contains(QStringLiteral("管路"))
        || simplified.contains(QStringLiteral("线缆"))
        || simplified.contains(QStringLiteral("pipeline"), Qt::CaseInsensitive)
        || simplified.contains(QStringLiteral("cable"), Qt::CaseInsensitive)
        || simplified.contains(QStringLiteral("hose"), Qt::CaseInsensitive)
        || simplified.contains(QStringLiteral("pipe"), Qt::CaseInsensitive);
}

bool ValidateOptions(const RobotCadAssemblyLoader::Options& options, QString& error)
{
    if (options.maximumFileBytes <= 0 || options.maximumFileBytes > kHardMaximumFileBytes)
    {
        error = QStringLiteral("机器人 STEP 文件大小限制无效：允许范围为 1 字节到 512 MiB。");
        return false;
    }
    if (options.buildDetailedPresentation
        && (options.displayBlockTargetFaceCount < 150
            || options.displayBlockTargetFaceCount > 300))
    {
        error = QStringLiteral("理论机器人显示块必须包含 150 到 300 个 B-Rep 面。");
        return false;
    }
    return true;
}

bool MergeBounds(
    const RobotCadAssemblyLoader::Bounds& source,
    RobotCadAssemblyLoader::Bounds& destination)
{
    if (!source.valid || !source.minimumMm.allFinite() || !source.maximumMm.allFinite()
        || (source.maximumMm.array() < source.minimumMm.array()).any())
    {
        return false;
    }
    if (!destination.valid)
    {
        destination = source;
        return true;
    }
    destination.minimumMm = destination.minimumMm.cwiseMin(source.minimumMm);
    destination.maximumMm = destination.maximumMm.cwiseMax(source.maximumMm);
    destination.valid = destination.minimumMm.allFinite()
        && destination.maximumMm.allFinite()
        && (destination.maximumMm.array() >= destination.minimumMm.array()).all();
    return destination.valid;
}

QString FileSha256(const QString& path, qint64 expectedSize, QString& error)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        error = QStringLiteral("无法读取机器人 STEP 以计算 SHA-256：%1")
            .arg(file.errorString());
        return QString();
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    qint64 bytesRead = 0;
    while (!file.atEnd())
    {
        const QByteArray chunk = file.read(1024 * 1024);
        if (chunk.isEmpty() && file.error() != QFileDevice::NoError)
        {
            error = QStringLiteral("读取机器人 STEP 计算 SHA-256 失败：%1")
                .arg(file.errorString());
            return QString();
        }
        bytesRead += chunk.size();
        hash.addData(chunk);
    }
    if (bytesRead != expectedSize)
    {
        error = QStringLiteral("机器人 STEP 在身份校验期间大小发生变化。");
        return QString();
    }
    return QString::fromLatin1(hash.result().toHex());
}
}

bool RobotCadAssemblyLoader::LoadFile(
    const QString& stepFilePath,
    Result& result,
    QString& error,
    const Options* suppliedOptions)
{
    QMutexLocker<QMutex> importLock(&occtsync::OperationMutex());

    result = Result();
    error.clear();
    const auto fail = [&result, &error](const QString& message)
    {
        result = Result();
        error = message;
        return false;
    };

    try
    {
        const Options options = suppliedOptions != nullptr ? *suppliedOptions : Options();
        if (!ValidateOptions(options, error)) return false;

        if (stepFilePath.isEmpty() || stepFilePath.contains(QChar::Null))
            return fail(QStringLiteral("机器人 STEP 文件路径为空或包含非法字符。"));

        const QFileInfo originalInfo(stepFilePath);
        if (!originalInfo.exists() || !originalInfo.isFile() || originalInfo.isSymLink())
            return fail(QStringLiteral("机器人 STEP 路径不是可读取的普通文件：%1").arg(stepFilePath));
        const QString extension = originalInfo.suffix().toLower();
        if (extension != QStringLiteral("step") && extension != QStringLiteral("stp"))
            return fail(QStringLiteral("机器人模型文件扩展名不是 .step 或 .stp：%1")
                            .arg(stepFilePath));

        const qint64 originalFileSize = originalInfo.size();
        const qint64 originalModifiedMs = originalInfo.lastModified().toMSecsSinceEpoch();
        if (originalFileSize <= 0 || originalFileSize > options.maximumFileBytes)
        {
            return fail(QStringLiteral("机器人 STEP 文件为空或超过大小限制（当前上限 %1 MiB）。")
                            .arg(options.maximumFileBytes / (1024LL * 1024LL)));
        }
        QString sourceHashError;
        const QString sourceSha256 = FileSha256(
            stepFilePath, originalFileSize, sourceHashError);
        if (sourceSha256.isEmpty()) return fail(sourceHashError);

#ifdef _WIN32
        const std::filesystem::path nativePath(stepFilePath.toStdWString());
#else
        const QByteArray encodedPath = stepFilePath.toUtf8();
        const std::filesystem::path nativePath(encodedPath.constData());
#endif
        std::ifstream input(nativePath, std::ios::in | std::ios::binary);
        if (!input.is_open() || input.fail())
            return fail(QStringLiteral("无法打开机器人 STEP 文件：%1").arg(stepFilePath));

        const bool namesAreUtf8 = IsValidUtf8(input);
        input.clear();
        input.seekg(0, std::ios::beg);
        if (input.fail())
            return fail(QStringLiteral("无法重新定位机器人 STEP 输入流。"));

        STEPCAFControl_Reader reader;
        // 某些国内厂商把 PRODUCT 名称直接写成 GB 字节而非 STEP Unicode 转义。
        // 只在当前 OCCT 操作锁内临时切换；关节识别仍只依赖末尾 ASCII J0...J6。
        ScopedStepCodePage codePage(namesAreUtf8 ? "UTF8" : "GB");
        if (!codePage.IsValid())
            return fail(QStringLiteral("无法配置 STEP 产品名称编码。"));
        reader.SetNameMode(Standard_True);
        reader.SetColorMode(Standard_False);
        reader.SetLayerMode(Standard_False);
        reader.SetPropsMode(Standard_False);
        const IFSelect_ReturnStatus readStatus = reader.ReadStream("robot.step", input);
        input.close();
        if (readStatus != IFSelect_RetDone) return fail(ReadFailureText(readStatus));

        Statistics importedStatistics;
        importedStatistics.sourceSha256 = sourceSha256;
        importedStatistics.occtVersion = QString::fromLatin1(OCC_VERSION_COMPLETE);
        importedStatistics.productNameEncoding = namesAreUtf8
            ? QStringLiteral("UTF-8")
            : QStringLiteral("GB fallback");
        importedStatistics.sourceLengthUnits = ReadLengthUnitNames(reader.ChangeReader());
        if (importedStatistics.sourceLengthUnits.isEmpty())
        {
            return fail(QStringLiteral(
                "机器人 STEP 文件没有可验证的长度单位，已拒绝把未知单位误当成毫米。"));
        }

        // OCCT 的系统长度单位 1.0 表示 1 mm；只修改当前 reader。
        reader.ChangeReader().SetSystemLengthUnit(1.0);
        if (reader.NbRootsForTransfer() <= 0)
            return fail(QStringLiteral("机器人 STEP 文件中没有可转换的产品根节点。"));

        DocumentGuard documentGuard;
        XCAFApp_Application::GetApplication()->NewDocument(
            TCollection_ExtendedString("MDTV-XCAF"), documentGuard.document);
        if (documentGuard.document.IsNull())
            return fail(QStringLiteral("无法创建机器人 STEP 装配文档。"));
        if (!reader.Transfer(documentGuard.document))
            return fail(QStringLiteral("机器人 STEP 装配结构转换失败。"));

        const Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(documentGuard.document->Main());
        if (shapeTool.IsNull())
            return fail(QStringLiteral("机器人 STEP 中没有可用的 XCAF 装配结构。"));

        TDF_LabelSequence freeLabels;
        shapeTool->GetFreeShapes(freeLabels);
        importedStatistics.freeShapeCount = static_cast<qsizetype>(freeLabels.Length());
        if (freeLabels.IsEmpty())
            return fail(QStringLiteral("机器人 STEP 装配中没有自由产品节点。"));

        std::array<SelectedComponent, 7> joints;
        std::array<bool, 7> jointFound{};
        std::vector<SelectedComponent> pipelines;

        XCAFPrs_DocumentExplorer explorer(
            documentGuard.document,
            XCAFPrs_DocumentExplorerFlags_NoStyle);
        for (; explorer.More(); explorer.Next())
        {
            const XCAFPrs_DocumentNode& node = explorer.Current();
            if (explorer.CurrentDepth() <= 0) continue;

            const QString instanceName = LabelName(node.Label);
            const QString productName = LabelName(
                node.RefLabel.IsNull() ? node.Label : node.RefLabel);
            const int jointIndex = JointIndexFromNames(productName, instanceName);
            const bool isPipeline = IsPipelineName(productName)
                || IsPipelineName(instanceName);

            ComponentStatistics component;
            component.assemblyPath = QString::fromLatin1(node.Id.ToCString());
            component.instanceName = instanceName;
            component.productName = productName;
            component.jointIndex = jointIndex;
            component.isPipeline = isPipeline;
            component.included = jointIndex >= 0 || (options.includePipeline && isPipeline);

            // 装配浏览始终记录全部组件身份。轻量模式的几何访问只发生在
            // 实际纳入的 J0...J6（以及显式选择的管线）上，大型工作空间/
            // 工装不再做 ShapeCount 或 ShapeBounds；详细模式保持原有统计。
            TopoDS_Shape componentShape;
            if (component.included || options.buildDetailedPresentation)
            {
                componentShape = XCAFPrs_DocumentExplorer::FindShapeFromPathId(
                    documentGuard.document, node.Id);
                component.solidCount = ShapeCount(componentShape, TopAbs_SOLID);
                component.faceCount = ShapeCount(componentShape, TopAbs_FACE);
                ShapeBounds(componentShape, component.boundsMm);
            }

            const qsizetype statisticsIndex = importedStatistics.components.size();
            importedStatistics.components.append(component);

            if (jointIndex >= 0)
            {
                if (componentShape.IsNull() || !component.boundsMm.valid)
                {
                    return fail(QStringLiteral("机器人组件 J%1 没有可用几何体或有限包围盒。")
                                    .arg(jointIndex));
                }
                if (jointFound[static_cast<size_t>(jointIndex)])
                {
                    return fail(QStringLiteral(
                        "机器人 STEP 中存在多个产品名末尾为 J%1 的装配实例，无法唯一选取。")
                                    .arg(jointIndex));
                }
                joints[static_cast<size_t>(jointIndex)] = { componentShape, statisticsIndex };
                jointFound[static_cast<size_t>(jointIndex)] = true;
            }
            else if (options.includePipeline && isPipeline
                     && !componentShape.IsNull() && component.boundsMm.valid)
            {
                pipelines.push_back({ componentShape, statisticsIndex });
            }
        }

        QStringList missingJoints;
        for (int jointIndex = 0; jointIndex <= 6; ++jointIndex)
        {
            if (!jointFound[static_cast<size_t>(jointIndex)])
                missingJoints.append(QStringLiteral("J%1").arg(jointIndex));
        }
        if (!missingJoints.isEmpty())
        {
            return fail(QStringLiteral(
                "机器人 STEP 未找到完整的 J0-J6 产品组件，缺少：%1。组件名必须以 J0...J6 结尾。")
                            .arg(missingJoints.join(QStringLiteral("、"))));
        }

        importedStatistics.discoveredComponentCount = importedStatistics.components.size();
        importedStatistics.jointComponentCount = 7;
        importedStatistics.includedPipelineCount = static_cast<qsizetype>(pipelines.size());
        importedStatistics.includedComponentCount = 7
            + importedStatistics.includedPipelineCount;

        // 两种结果都严格保留 J0...J6（和可选管线）的组件级统计。轻量模式
        // 直接合并有限 AABB 与计数，不构造完整 compound。
        for (int jointIndex = 0; jointIndex <= 6; ++jointIndex)
        {
            const SelectedComponent& joint = joints[static_cast<size_t>(jointIndex)];
            const ComponentStatistics& component =
                importedStatistics.components[joint.statisticsIndex];
            if (!MergeBounds(component.boundsMm, importedStatistics.assemblyBoundsMm))
                return fail(QStringLiteral("J%1 的机器人组件包围盒无法合并。").arg(jointIndex));
            importedStatistics.solidCount += component.solidCount;
            importedStatistics.faceCount += component.faceCount;
        }
        for (const SelectedComponent& pipeline : pipelines)
        {
            const ComponentStatistics& component =
                importedStatistics.components[pipeline.statisticsIndex];
            if (!MergeBounds(component.boundsMm, importedStatistics.assemblyBoundsMm))
                return fail(QStringLiteral("机器人管线组件包围盒无法合并。"));
            importedStatistics.solidCount += component.solidCount;
            importedStatistics.faceCount += component.faceCount;
        }
        if (!importedStatistics.assemblyBoundsMm.valid)
            return fail(QStringLiteral("J0-J6 合并后的机器人装配包围盒为空或无效。"));

        std::shared_ptr<TopoDS_Shape> assemblyShape;
        std::shared_ptr<TopoDS_Shape> j0Shape;
        std::vector<std::shared_ptr<TopoDS_Shape>> displayBlocks;
        if (options.buildDetailedPresentation)
        {
            BRep_Builder builder;
            TopoDS_Compound assembly;
            builder.MakeCompound(assembly);
            for (const SelectedComponent& joint : joints) builder.Add(assembly, joint.shape);
            for (const SelectedComponent& pipeline : pipelines)
                builder.Add(assembly, pipeline.shape);

            // 详细模式继续以 compound 的唯一子形状计数为准，完全保留原有
            // B-Rep 后备行为；轻量模式则不会进入本分支。
            if (!ShapeBounds(assembly, importedStatistics.assemblyBoundsMm))
                return fail(QStringLiteral("J0-J6 合并后的机器人装配包围盒为空或无效。"));
            importedStatistics.solidCount = ShapeCount(assembly, TopAbs_SOLID);
            importedStatistics.faceCount = ShapeCount(assembly, TopAbs_FACE);

            if (options.prepareDisplayTriangulation)
            {
                const Eigen::Vector3d span = importedStatistics.assemblyBoundsMm.maximumMm
                    - importedStatistics.assemblyBoundsMm.minimumMm;
                const double deflectionMm = std::clamp(span.norm() * 0.0006, 0.5, 1.5);
                BRepMesh_IncrementalMesh mesher(
                    assembly,
                    deflectionMm,
                    Standard_False,
                    0.25,
                    Standard_True);
                mesher.Perform();
                if (!mesher.IsDone())
                    return fail(QStringLiteral("理论机器人 B-Rep 显示缓存生成失败。"));
                importedStatistics.displayTriangulationPrepared = true;
            }

            // AIS_Shape 对整个 2.6 万面总装的一次 Compute 会长时间占用 GUI
            // 线程。这里仅把原始 B-Rep face 引用分装到小 compound 中；face 的
            // TShape、Location 以及上面刚生成的 OCCT 内存三角化缓存都被共享，
            // 不发生几何转换，也不会生成或保存替代网格文件。
            TopTools_IndexedMapOfShape displayFaces;
            TopExp::MapShapes(assembly, TopAbs_FACE, displayFaces);
            if (displayFaces.Extent() != importedStatistics.faceCount)
                return fail(QStringLiteral("理论机器人显示分块的 B-Rep 面计数不一致。"));
            displayBlocks.reserve(static_cast<size_t>(
                (displayFaces.Extent() + options.displayBlockTargetFaceCount - 1)
                / options.displayBlockTargetFaceCount));
            for (Standard_Integer first = 1; first <= displayFaces.Extent();)
            {
                TopoDS_Compound block;
                BRep_Builder blockBuilder;
                blockBuilder.MakeCompound(block);
                const Standard_Integer last = std::min<Standard_Integer>(
                    displayFaces.Extent(),
                    first + static_cast<Standard_Integer>(
                        options.displayBlockTargetFaceCount) - 1);
                for (Standard_Integer index = first; index <= last; ++index)
                    blockBuilder.Add(block, displayFaces.FindKey(index));
                displayBlocks.push_back(std::make_shared<TopoDS_Shape>(block));
                first = last + 1;
            }
            if (displayBlocks.empty())
                return fail(QStringLiteral("理论机器人显示分块为空。"));
            importedStatistics.displayBlockCount =
                static_cast<qsizetype>(displayBlocks.size());
            importedStatistics.detailedPresentationBuilt = true;
            assemblyShape = std::make_shared<TopoDS_Shape>(assembly);
            j0Shape = std::make_shared<TopoDS_Shape>(joints[0].shape);
        }

        BaseGeometry base;
        base.sourceUp = Eigen::Vector3d::UnitY();
        base.j0BoundsMm = importedStatistics.components[
            joints[0].statisticsIndex].boundsMm;
        if (!base.j0BoundsMm.valid)
            return fail(QStringLiteral("J0 基座包围盒为空或无效。"));
        base.minimumYmm = base.j0BoundsMm.minimumMm.y();
        base.conservativeBaseCenterMm = Eigen::Vector3d(
            0.5 * (base.j0BoundsMm.minimumMm.x() + base.j0BoundsMm.maximumMm.x()),
            base.minimumYmm,
            0.5 * (base.j0BoundsMm.minimumMm.z() + base.j0BoundsMm.maximumMm.z()));
        base.valid = base.conservativeBaseCenterMm.allFinite();
        if (!base.valid) return fail(QStringLiteral("J0 基座中心计算结果无效。"));

        QFileInfo finalInfoBeforeHash(stepFilePath);
        finalInfoBeforeHash.refresh();
        if (!finalInfoBeforeHash.exists() || !finalInfoBeforeHash.isFile()
            || finalInfoBeforeHash.isSymLink()
            || finalInfoBeforeHash.size() != originalFileSize
            || finalInfoBeforeHash.lastModified().toMSecsSinceEpoch() != originalModifiedMs)
        {
            return fail(QStringLiteral("机器人 STEP 文件在导入过程中发生变化，请重新导入。"));
        }
        QString finalHashError;
        const QString finalSha256 = FileSha256(
            stepFilePath, originalFileSize, finalHashError);
        if (finalSha256.isEmpty()) return fail(finalHashError);

        QFileInfo finalInfoAfterHash(stepFilePath);
        finalInfoAfterHash.refresh();
        if (!finalInfoAfterHash.exists() || !finalInfoAfterHash.isFile()
            || finalInfoAfterHash.isSymLink()
            || finalInfoAfterHash.size() != originalFileSize
            || finalInfoAfterHash.lastModified().toMSecsSinceEpoch() != originalModifiedMs)
        {
            return fail(QStringLiteral(
                "机器人 STEP 文件在导入后的内容复核期间发生变化，请重新导入。"));
        }
        if (finalSha256 != sourceSha256)
        {
            return fail(QStringLiteral(
                "机器人 STEP 内容在导入过程中发生变化（即使大小或修改时间未变），请重新导入。"));
        }

        Result importedResult;
        importedResult.assemblyShape = std::move(assemblyShape);
        importedResult.j0Shape = std::move(j0Shape);
        importedResult.displayBlocks = std::move(displayBlocks);
        importedResult.base = base;
        importedResult.statistics = std::move(importedStatistics);
        result = std::move(importedResult);
        return true;
    }
    catch (const Standard_Failure& exception)
    {
        const QString detail = ExceptionDetail(exception.GetMessageString());
        return fail(detail.isEmpty()
                ? QStringLiteral("Open CASCADE 在读取机器人 STEP 装配时发生异常。")
                : QStringLiteral("Open CASCADE 在读取机器人 STEP 装配时发生异常：%1")
                      .arg(detail));
    }
    catch (const std::bad_alloc&)
    {
        return fail(QStringLiteral("读取机器人 STEP 装配时内存不足，模型可能过于复杂。"));
    }
    catch (const std::exception& exception)
    {
        const QString detail = ExceptionDetail(exception.what());
        return fail(detail.isEmpty()
                ? QStringLiteral("读取机器人 STEP 装配时发生标准库异常。")
                : QStringLiteral("读取机器人 STEP 装配时发生异常：%1").arg(detail));
    }
    catch (...)
    {
        return fail(QStringLiteral("读取机器人 STEP 装配时发生未知异常。"));
    }
}
