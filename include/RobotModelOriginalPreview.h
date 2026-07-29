#pragma once

#include <QImage>
#include <QString>

// 从受控的原始机器人 STEP 总装生成服务器模型缩略图。
//
// 这里使用 RobotCadAssemblyLoader 保留的原始 B-Rep 及其显示三角化缓存，
// 只把实际 CAD 曲面软件光栅化为固定尺寸 PNG；不读取碰撞 AABB，也不以简模
// 作为失败回退，避免模型列表把碰撞包围盒误当作机器人外形。
class RobotModelOriginalPreview final
{
public:
    static bool RenderStepFile(
        const QString& stepFilePath,
        const QString& displayName,
        QImage& image,
        QString& error);

private:
    RobotModelOriginalPreview() = delete;
};
