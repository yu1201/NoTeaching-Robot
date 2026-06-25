#pragma once

// 共享的 3D 点云查看控件。
// 原本内联在 QtWidgetsApplication4.cpp 的匿名命名空间里，现抽取到 pcview 命名空间，
// 供 PointCloudViewerDialog（点云查看）与 WeldSeamCompDialog（补偿前后可视化）等复用。
// 该控件无 Q_OBJECT，无信号槽，纯软件正交投影渲染。

#include <QApplication>
#include <QColor>
#include <QEvent>
#include <QFont>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QTouchEvent>
#include <QVector>
#include <QWheelEvent>
#include <QWidget>

#include <algorithm>
#include <cmath>

namespace pcview
{
    struct PointCloudVec3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    inline PointCloudVec3 operator+(const PointCloudVec3& a, const PointCloudVec3& b)
    {
        return { a.x + b.x, a.y + b.y, a.z + b.z };
    }

    inline PointCloudVec3 operator-(const PointCloudVec3& a, const PointCloudVec3& b)
    {
        return { a.x - b.x, a.y - b.y, a.z - b.z };
    }

    inline PointCloudVec3 operator*(const PointCloudVec3& point, double scale)
    {
        return { point.x * scale, point.y * scale, point.z * scale };
    }

    inline double DotPoint3D(const PointCloudVec3& a, const PointCloudVec3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline double LengthPoint3D(const PointCloudVec3& point)
    {
        return std::sqrt(std::max(0.0, DotPoint3D(point, point)));
    }

    inline PointCloudVec3 NormalizePoint3D(const PointCloudVec3& point)
    {
        const double length = LengthPoint3D(point);
        if (length <= 1.0e-9)
        {
            return { 0.0, 0.0, 0.0 };
        }
        return point * (1.0 / length);
    }

    inline PointCloudVec3 CrossPoint3D(const PointCloudVec3& a, const PointCloudVec3& b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x
        };
    }

    inline PointCloudVec3 RotatePoint3D(const PointCloudVec3& value, PointCloudVec3 axis, double radians)
    {
        axis = NormalizePoint3D(axis);
        if (LengthPoint3D(axis) <= 1.0e-9)
        {
            return value;
        }
        const double c = std::cos(radians);
        const double s = std::sin(radians);
        return value * c + CrossPoint3D(axis, value) * s + axis * (DotPoint3D(axis, value) * (1.0 - c));
    }

    struct LoadedPointCloudFile
    {
        QVector<PointCloudVec3> points;
        QString error;
        int skippedLineCount = 0;
    };

    // 从空格/逗号分隔的 x/y/z 文本文件读取点云（带可选表头识别）。定义在 PointCloud3DView.cpp。
    LoadedPointCloudFile LoadPointCloudFile3D(const QString& filePath);

    class PointCloud3DView final : public QWidget
    {
    public:
        struct Layer
        {
            QString name;
            QString path;
            QVector<PointCloudVec3> points;
            QColor color;
            bool visible = true;
            bool rainbow = false;
            bool connectLines = false;
            double pointSize = 0.0;  // 点宽(px)，0=自动（连线 2.4 / 不连线 2.0）
        };

        // 世界空间方向箭头，用于标注补偿正负影响方向。origin/vector 单位与点云一致（mm）。
        struct DirectionArrow
        {
            PointCloudVec3 origin;
            PointCloudVec3 vector;
            QString label;
            QColor color = QColor(255, 215, 64);
            bool doubleHeaded = false; // true 时从 origin-vector 画到 origin+vector，两端都画箭头
        };

        explicit PointCloud3DView(QWidget* parent = nullptr)
            : QWidget(parent)
        {
            setMinimumSize(720, 520);
            setMouseTracking(true);
            setFocusPolicy(Qt::StrongFocus);
            setAttribute(Qt::WA_AcceptTouchEvents, true);
            SetTopView(false);
        }

        void SetLayers(const QVector<Layer>& layers)
        {
            m_layers = layers;
            ClearSelectedRotationCenter(false);
            FitToLayers();
            update();
        }

        // 仅更新图层外观(颜色/连线/点大小/可见)而不动相机视角与旋转中心——供属性面板实时调样式用。
        void SetLayersPreserveView(const QVector<Layer>& layers)
        {
            const int prevCount = m_layers.size();
            m_layers = layers;
            if (m_layers.size() != prevCount)
            {
                ClearSelectedRotationCenter(false);  // 图层增删才重置选中点，纯改样式不动
            }
            update();
        }

        void SetLayerVisible(int index, bool visible)
        {
            if (index < 0 || index >= m_layers.size())
            {
                return;
            }
            m_layers[index].visible = visible;
            if (!visible && m_selectedLayerIndex == index)
            {
                ClearSelectedRotationCenter(false);
            }
            update();
        }

        void SetDirectionArrows(const QVector<DirectionArrow>& arrows)
        {
            m_arrows = arrows;
            update();
        }

        void ResetView()
        {
            m_zoom = 1.0;
            m_pan = QPointF();
            ClearSelectedRotationCenter(false);
            FitToLayers(false);
            update();
        }

        void SetTopView(bool updateNow = true)
        {
            m_right = { 1.0, 0.0, 0.0 };
            m_up = { 0.0, 1.0, 0.0 };
            m_forward = { 0.0, 0.0, 1.0 };
            if (updateNow)
            {
                update();
            }
        }

        void SetFrontView()
        {
            m_right = { 1.0, 0.0, 0.0 };
            m_up = { 0.0, 0.0, 1.0 };
            m_forward = { 0.0, -1.0, 0.0 };
            update();
        }

        void SetCameraLikeView()
        {
            m_right = NormalizePoint3D({ 0.86, 0.48, 0.0 });
            m_up = NormalizePoint3D({ -0.18, 0.32, 0.93 });
            m_forward = NormalizePoint3D(CrossPoint3D(m_right, m_up));
            update();
        }

    protected:
        bool event(QEvent* event) override
        {
            if (event != nullptr)
            {
                switch (event->type())
                {
                case QEvent::TouchBegin:
                case QEvent::TouchUpdate:
                case QEvent::TouchEnd:
                case QEvent::TouchCancel:
                    return HandleTouchEvent(static_cast<QTouchEvent*>(event));
                default:
                    break;
                }
            }
            return QWidget::event(event);
        }

        void paintEvent(QPaintEvent* event) override
        {
            Q_UNUSED(event);

            QPainter painter(this);
            painter.setRenderHint(QPainter::Antialiasing, false);
            QLinearGradient background(rect().topLeft(), rect().bottomLeft());
            background.setColorAt(0.0, QColor(12, 97, 130));
            background.setColorAt(0.62, QColor(5, 47, 64));
            background.setColorAt(1.0, QColor(0, 0, 0));
            painter.fillRect(rect(), background);

            DrawGrid(painter);
            DrawLayers(painter);
            DrawSelectedRotationCenter(painter);
            DrawCrosshair(painter);
            DrawScaleBar(painter);
            DrawAxes(painter);
            DrawDirectionArrows(painter);
            DrawOverlayText(painter);
        }

        void wheelEvent(QWheelEvent* event) override
        {
            const int delta = event->angleDelta().y();
            if (delta == 0)
            {
                QWidget::wheelEvent(event);
                return;
            }
            const double factor = delta > 0 ? 1.18 : (1.0 / 1.18);
            m_zoom = std::clamp(m_zoom * factor, 0.05, 120.0);
            event->accept();
            update();
        }

        void mousePressEvent(QMouseEvent* event) override
        {
            m_lastMousePos = event->position();
            m_pressMousePos = event->position();
            m_leftDragActive = false;
            if (event->button() == Qt::LeftButton)
            {
                m_rotating = true;
                setCursor(Qt::ClosedHandCursor);
                event->accept();
                return;
            }
            if (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton)
            {
                m_panning = true;
                setCursor(Qt::SizeAllCursor);
                event->accept();
                return;
            }
            QWidget::mousePressEvent(event);
        }

        void mouseMoveEvent(QMouseEvent* event) override
        {
            const QPointF delta = event->position() - m_lastMousePos;
            m_lastMousePos = event->position();
            if (m_rotating)
            {
                if (!m_leftDragActive)
                {
                    const QPointF totalDelta = event->position() - m_pressMousePos;
                    if (std::hypot(totalDelta.x(), totalDelta.y()) < QApplication::startDragDistance())
                    {
                        event->accept();
                        return;
                    }
                    m_leftDragActive = true;
                }
                const double yaw = delta.x() * 0.008;
                const double pitch = delta.y() * 0.008;
                m_right = RotatePoint3D(m_right, m_up, yaw);
                m_forward = RotatePoint3D(m_forward, m_up, yaw);
                m_up = RotatePoint3D(m_up, m_right, pitch);
                m_forward = RotatePoint3D(m_forward, m_right, pitch);
                NormalizeBasis();
                event->accept();
                update();
                return;
            }
            if (m_panning)
            {
                const double scale = EffectiveScale();
                m_pan += QPointF(delta.x() / scale, -delta.y() / scale);
                event->accept();
                update();
                return;
            }
            QWidget::mouseMoveEvent(event);
        }

        void mouseReleaseEvent(QMouseEvent* event) override
        {
            if (event->button() == Qt::LeftButton && m_rotating)
            {
                const bool wasDrag = m_leftDragActive;
                m_rotating = false;
                m_leftDragActive = false;
                unsetCursor();
                if (!wasDrag)
                {
                    SelectRotationCenterAt(event->position(), 14.0);
                }
                event->accept();
                return;
            }
            if ((event->button() == Qt::MiddleButton || event->button() == Qt::RightButton) && m_panning)
            {
                m_panning = false;
                unsetCursor();
                event->accept();
                return;
            }
            QWidget::mouseReleaseEvent(event);
        }

        void mouseDoubleClickEvent(QMouseEvent* event) override
        {
            if (event->button() == Qt::LeftButton)
            {
                ResetView();
                event->accept();
                return;
            }
            QWidget::mouseDoubleClickEvent(event);
        }

    private:
        void NormalizeBasis()
        {
            m_right = NormalizePoint3D(m_right);
            m_up = NormalizePoint3D(m_up - m_right * DotPoint3D(m_up, m_right));
            m_forward = NormalizePoint3D(CrossPoint3D(m_right, m_up));
        }

        PointCloudVec3 RotationCenter() const
        {
            return HasSelectedRotationCenter() ? m_selectedRotationCenter : m_center;
        }

        bool HasSelectedRotationCenter() const
        {
            return m_selectedLayerIndex >= 0
                && m_selectedLayerIndex < m_layers.size()
                && m_selectedPointIndex >= 0
                && m_selectedPointIndex < m_layers.at(m_selectedLayerIndex).points.size()
                && m_layers.at(m_selectedLayerIndex).visible;
        }

        void ClearSelectedRotationCenter(bool updateNow = true)
        {
            m_selectedLayerIndex = -1;
            m_selectedPointIndex = -1;
            m_selectedRotationCenter = {};
            if (updateNow)
            {
                update();
            }
        }

        void FitToLayers(bool resetZoomAndPan = true)
        {
            bool hasBounds = false;
            PointCloudVec3 minPoint;
            PointCloudVec3 maxPoint;
            for (const Layer& layer : m_layers)
            {
                for (const PointCloudVec3& point : layer.points)
                {
                    if (!hasBounds)
                    {
                        minPoint = maxPoint = point;
                        hasBounds = true;
                    }
                    else
                    {
                        minPoint.x = std::min(minPoint.x, point.x);
                        minPoint.y = std::min(minPoint.y, point.y);
                        minPoint.z = std::min(minPoint.z, point.z);
                        maxPoint.x = std::max(maxPoint.x, point.x);
                        maxPoint.y = std::max(maxPoint.y, point.y);
                        maxPoint.z = std::max(maxPoint.z, point.z);
                    }
                }
            }

            if (!hasBounds)
            {
                m_center = { 0.0, 0.0, 0.0 };
                m_baseSpan = 200.0;
            }
            else
            {
                m_center = (minPoint + maxPoint) * 0.5;
                const PointCloudVec3 span = maxPoint - minPoint;
                m_baseSpan = std::max({ std::abs(span.x), std::abs(span.y), std::abs(span.z), 10.0 });
            }
            if (resetZoomAndPan)
            {
                m_zoom = 1.0;
                m_pan = QPointF();
            }
        }

        double EffectiveScale() const
        {
            const double screenSpan = std::max(1, std::min(width(), height()) - 80);
            return screenSpan / std::max(1.0, m_baseSpan) * 0.92 * m_zoom;
        }

        QPointF ProjectPoint(const PointCloudVec3& point) const
        {
            const PointCloudVec3 relative = point - RotationCenter();
            const double scale = EffectiveScale();
            const double sx = DotPoint3D(relative, m_right) + m_pan.x();
            const double sy = DotPoint3D(relative, m_up) + m_pan.y();
            return QPointF(width() * 0.5 + sx * scale, height() * 0.5 - sy * scale);
        }

        void KeepPointAtScreenPosition(const PointCloudVec3& point, const QPointF& targetScreenPos)
        {
            const double scale = EffectiveScale();
            if (scale <= 1.0e-9)
            {
                return;
            }
            const QPointF currentScreenPos = ProjectPoint(point);
            const QPointF screenDelta = targetScreenPos - currentScreenPos;
            m_pan += QPointF(screenDelta.x() / scale, -screenDelta.y() / scale);
        }

        bool FindNearestVisiblePoint(
            const QPointF& screenPos,
            double maxDistance,
            int& layerIndex,
            int& pointIndex,
            PointCloudVec3& point) const
        {
            double bestDistance2 = maxDistance * maxDistance;
            bool found = false;
            for (int candidateLayerIndex = 0; candidateLayerIndex < m_layers.size(); ++candidateLayerIndex)
            {
                const Layer& layer = m_layers.at(candidateLayerIndex);
                if (!layer.visible || layer.points.isEmpty())
                {
                    continue;
                }
                for (int candidatePointIndex = 0; candidatePointIndex < layer.points.size(); ++candidatePointIndex)
                {
                    const QPointF projected = ProjectPoint(layer.points.at(candidatePointIndex));
                    const double dx = projected.x() - screenPos.x();
                    const double dy = projected.y() - screenPos.y();
                    const double distance2 = dx * dx + dy * dy;
                    if (distance2 <= bestDistance2)
                    {
                        bestDistance2 = distance2;
                        layerIndex = candidateLayerIndex;
                        pointIndex = candidatePointIndex;
                        point = layer.points.at(candidatePointIndex);
                        found = true;
                    }
                }
            }
            return found;
        }

        void SelectRotationCenterAt(const QPointF& screenPos, double maxDistance)
        {
            int layerIndex = -1;
            int pointIndex = -1;
            PointCloudVec3 point;
            if (!FindNearestVisiblePoint(screenPos, maxDistance, layerIndex, pointIndex, point))
            {
                return;
            }
            const QPointF oldScreenPos = ProjectPoint(point);
            m_selectedLayerIndex = layerIndex;
            m_selectedPointIndex = pointIndex;
            m_selectedRotationCenter = point;
            KeepPointAtScreenPosition(point, oldScreenPos);
            update();
        }

        void DrawGrid(QPainter& painter) const
        {
            QPen gridPen(QColor(255, 255, 255, 85));
            gridPen.setStyle(Qt::DotLine);
            gridPen.setWidthF(1.0);
            painter.setPen(gridPen);
            const int step = std::max(80, std::min(width(), height()) / 4);
            for (int x = width() / 2 % step; x < width(); x += step)
            {
                painter.drawLine(x, 0, x, height());
            }
            for (int y = height() / 2 % step; y < height(); y += step)
            {
                painter.drawLine(0, y, width(), y);
            }

            QPen axisPen(QColor(255, 255, 255, 145));
            axisPen.setWidthF(1.0);
            painter.setPen(axisPen);
            painter.drawLine(width() / 2, 0, width() / 2, height());
            painter.drawLine(0, height() / 2, width(), height() / 2);
        }

        static QColor LayerPointColor(const Layer& layer, int pointIndex, int pointCount)
        {
            if (!layer.rainbow || pointCount <= 1)
            {
                return layer.color;
            }
            const double hue = std::clamp(static_cast<double>(pointIndex) / static_cast<double>(pointCount - 1) * 0.72, 0.0, 0.72);
            return QColor::fromHsvF(hue, 1.0, 1.0);
        }

        void DrawLayers(QPainter& painter) const
        {
            for (const Layer& layer : m_layers)
            {
                if (!layer.visible || layer.points.isEmpty())
                {
                    continue;
                }
                const int pointCount = layer.points.size();
                const int drawStep = std::max(1, pointCount / 90000);
                if (layer.connectLines && pointCount > 1)
                {
                    QPen linePen(layer.color);
                    linePen.setWidthF(1.2);
                    linePen.setCosmetic(true);
                    painter.setPen(linePen);
                    QPointF previous = ProjectPoint(layer.points.at(0));
                    for (int index = drawStep; index < pointCount; index += drawStep)
                    {
                        const QPointF current = ProjectPoint(layer.points.at(index));
                        painter.drawLine(previous, current);
                        previous = current;
                    }
                }

                QPen pointPen(layer.color);
                pointPen.setWidthF(layer.pointSize > 0.0
                    ? layer.pointSize
                    : (layer.connectLines ? 2.4 : 2.0));
                pointPen.setCosmetic(true);
                painter.setPen(pointPen);
                for (int index = 0; index < pointCount; index += drawStep)
                {
                    if (layer.rainbow)
                    {
                        pointPen.setColor(LayerPointColor(layer, index, pointCount));
                        painter.setPen(pointPen);
                    }
                    painter.drawPoint(ProjectPoint(layer.points.at(index)));
                }
            }
        }

        void DrawSelectedRotationCenter(QPainter& painter) const
        {
            if (!HasSelectedRotationCenter())
            {
                return;
            }
            const QPointF screenPos = ProjectPoint(m_selectedRotationCenter);
            painter.save();
            painter.setRenderHint(QPainter::Antialiasing, true);
            QPen haloPen(QColor(255, 255, 255, 230));
            haloPen.setWidthF(2.0);
            haloPen.setCosmetic(true);
            painter.setPen(haloPen);
            painter.setBrush(QColor(255, 90, 75, 210));
            painter.drawEllipse(screenPos, 7.0, 7.0);
            QPen crossPen(QColor(255, 255, 255, 245));
            crossPen.setWidthF(1.2);
            crossPen.setCosmetic(true);
            painter.setPen(crossPen);
            painter.drawLine(screenPos + QPointF(-11.0, 0.0), screenPos + QPointF(11.0, 0.0));
            painter.drawLine(screenPos + QPointF(0.0, -11.0), screenPos + QPointF(0.0, 11.0));
            painter.restore();
        }

        void DrawCrosshair(QPainter& painter) const
        {
            QPen pen(QColor(255, 255, 255, 180));
            pen.setWidthF(1.0);
            painter.setPen(pen);
            const QPointF center(width() * 0.5, height() * 0.5);
            painter.drawLine(center + QPointF(-8, 0), center + QPointF(8, 0));
            painter.drawLine(center + QPointF(0, -8), center + QPointF(0, 8));
        }

        void DrawScaleBar(QPainter& painter) const
        {
            const double scale = EffectiveScale();
            if (scale <= 1.0e-9)
            {
                return;
            }
            const double desiredPixel = std::max(120.0, width() * 0.18);
            const double rawWorldLength = desiredPixel / scale;
            const double magnitude = std::pow(10.0, std::floor(std::log10(std::max(1.0, rawWorldLength))));
            double worldLength = magnitude;
            if (rawWorldLength / magnitude >= 5.0)
            {
                worldLength = 5.0 * magnitude;
            }
            else if (rawWorldLength / magnitude >= 2.0)
            {
                worldLength = 2.0 * magnitude;
            }
            const double pixelLength = worldLength * scale;
            const QPointF start(width() - pixelLength - 70.0, height() - 34.0);
            const QPointF end(width() - 70.0, height() - 34.0);
            QPen pen(QColor(245, 245, 245));
            pen.setWidthF(1.5);
            painter.setPen(pen);
            painter.drawLine(start, end);
            painter.drawLine(start + QPointF(0, -4), start + QPointF(0, 4));
            painter.drawLine(end + QPointF(0, -4), end + QPointF(0, 4));
            QFont font = painter.font();
            font.setPointSize(10);
            font.setBold(true);
            painter.setFont(font);
            painter.drawText(QRectF(start.x(), start.y() + 4, pixelLength, 24), Qt::AlignCenter, QString::number(worldLength, 'f', worldLength >= 10.0 ? 0 : 1));
        }

        void DrawAxes(QPainter& painter) const
        {
            const QPointF origin(width() - 58.0, height() - 52.0);
            const double axisLength = 34.0;
            auto drawAxis = [&](const PointCloudVec3& axis, const QColor& color, const QString& label)
            {
                const QPointF direction(DotPoint3D(axis, m_right), -DotPoint3D(axis, m_up));
                const double length = std::max(1.0, std::sqrt(direction.x() * direction.x() + direction.y() * direction.y()));
                const QPointF normalized(direction.x() / length, direction.y() / length);
                QPen pen(color);
                pen.setWidthF(2.0);
                painter.setPen(pen);
                const QPointF end = origin + normalized * axisLength;
                painter.drawLine(origin, end);
                QFont font = painter.font();
                font.setPointSize(10);
                font.setBold(true);
                painter.setFont(font);
                painter.drawText(end + normalized * 7.0, label);
            };
            drawAxis({ 1.0, 0.0, 0.0 }, QColor(255, 40, 40), "X");
            drawAxis({ 0.0, 1.0, 0.0 }, QColor(40, 255, 40), "Y");
            drawAxis({ 0.0, 0.0, 1.0 }, QColor(0, 180, 255), "Z");
        }

        void DrawDirectionArrows(QPainter& painter) const
        {
            if (m_arrows.isEmpty())
            {
                return;
            }
            painter.save();
            painter.setRenderHint(QPainter::Antialiasing, true);
            QFont font = painter.font();
            font.setPointSize(9);
            font.setBold(true);
            painter.setFont(font);
            for (const DirectionArrow& arrow : m_arrows)
            {
                const PointCloudVec3 tail = arrow.doubleHeaded ? (arrow.origin - arrow.vector) : arrow.origin;
                const QPointF p0 = ProjectPoint(tail);
                const QPointF p1 = ProjectPoint(arrow.origin + arrow.vector);
                const QPointF dir = p1 - p0;
                const double length = std::hypot(dir.x(), dir.y());
                if (length < 1.0)
                {
                    continue;
                }
                QPen pen(arrow.color);
                pen.setWidthF(2.2);
                pen.setCosmetic(true);
                painter.setPen(pen);
                painter.drawLine(p0, p1);

                const QPointF unit(dir.x() / length, dir.y() / length);
                const QPointF normal(-unit.y(), unit.x());
                const double head = 9.0;
                const auto drawHead = [&](const QPointF& tip, const QPointF& back)
                {
                    painter.drawLine(tip, back + normal * (head * 0.5));
                    painter.drawLine(tip, back - normal * (head * 0.5));
                };
                drawHead(p1, p1 - unit * head);
                if (arrow.doubleHeaded)
                {
                    drawHead(p0, p0 + unit * head);
                }
                if (!arrow.label.isEmpty())
                {
                    painter.drawText(p1 + unit * 6.0 + QPointF(2.0, -2.0), arrow.label);
                }
            }
            painter.restore();
        }

        void DrawOverlayText(QPainter& painter) const
        {
            int visibleLayerCount = 0;
            int pointCount = 0;
            for (const Layer& layer : m_layers)
            {
                if (layer.visible)
                {
                    ++visibleLayerCount;
                    pointCount += layer.points.size();
                }
            }
            QFont font = painter.font();
            font.setPointSize(10);
            painter.setFont(font);
            painter.setPen(QColor(235, 245, 246));
            painter.drawText(QRectF(12, 10, width() - 24, 22), Qt::AlignLeft | Qt::AlignVCenter,
                QString("图层：%1  点数：%2  单击点设旋转中心 / 左键或单指旋转 / 双指平移缩放 / 双击复位")
                .arg(visibleLayerCount)
                .arg(pointCount));
            if (HasSelectedRotationCenter())
            {
                const QString layerName = m_layers.at(m_selectedLayerIndex).name;
                painter.drawText(QRectF(12, 32, width() - 24, 22), Qt::AlignLeft | Qt::AlignVCenter,
                    QString("旋转中心：%1 #%2  X=%3  Y=%4  Z=%5")
                    .arg(layerName)
                    .arg(m_selectedPointIndex + 1)
                    .arg(m_selectedRotationCenter.x, 0, 'f', 2)
                    .arg(m_selectedRotationCenter.y, 0, 'f', 2)
                    .arg(m_selectedRotationCenter.z, 0, 'f', 2));
            }
        }

        QVector<QPointF> ActiveTouchPositions(const QTouchEvent* event) const
        {
            QVector<QPointF> positions;
            if (event == nullptr)
            {
                return positions;
            }
            for (const QEventPoint& point : event->points())
            {
                if (point.state() != QEventPoint::State::Released)
                {
                    positions.push_back(point.position());
                }
            }
            return positions;
        }

        static QPointF TouchCenter(const QPointF& first, const QPointF& second)
        {
            return QPointF((first.x() + second.x()) * 0.5, (first.y() + second.y()) * 0.5);
        }

        static double TouchDistance(const QPointF& first, const QPointF& second)
        {
            return std::hypot(first.x() - second.x(), first.y() - second.y());
        }

        void BeginSingleTouch(const QPointF& pos, bool allowTapSelection = true)
        {
            m_touchMode = TouchMode::Single;
            m_touchStartPos = pos;
            m_lastTouchPos = pos;
            m_touchSingleDragActive = false;
            m_allowTouchTapSelection = allowTapSelection;
        }

        void BeginTwoFingerTouch(const QPointF& first, const QPointF& second)
        {
            m_touchMode = TouchMode::TwoFinger;
            m_lastTouchCenter = TouchCenter(first, second);
            m_lastTouchDistance = TouchDistance(first, second);
            m_touchSingleDragActive = false;
            m_allowTouchTapSelection = false;
        }

        void ResetTouchState()
        {
            m_touchMode = TouchMode::None;
            m_touchSingleDragActive = false;
            m_allowTouchTapSelection = false;
            m_lastTouchDistance = 0.0;
        }

        void RotateByScreenDelta(const QPointF& delta)
        {
            const double yaw = delta.x() * 0.008;
            const double pitch = delta.y() * 0.008;
            m_right = RotatePoint3D(m_right, m_up, yaw);
            m_forward = RotatePoint3D(m_forward, m_up, yaw);
            m_up = RotatePoint3D(m_up, m_right, pitch);
            m_forward = RotatePoint3D(m_forward, m_right, pitch);
            NormalizeBasis();
        }

        void PanAndZoomFromTouch(const QPointF& currentCenter, double currentDistance)
        {
            const double oldScale = EffectiveScale();
            if (oldScale <= 1.0e-9)
            {
                return;
            }
            const double oldViewX = (m_lastTouchCenter.x() - width() * 0.5) / oldScale - m_pan.x();
            const double oldViewY = -(m_lastTouchCenter.y() - height() * 0.5) / oldScale - m_pan.y();
            if (m_lastTouchDistance > 1.0 && currentDistance > 1.0)
            {
                const double factor = std::clamp(currentDistance / m_lastTouchDistance, 0.35, 2.85);
                m_zoom = std::clamp(m_zoom * factor, 0.05, 120.0);
            }
            const double newScale = EffectiveScale();
            if (newScale <= 1.0e-9)
            {
                return;
            }
            m_pan.setX((currentCenter.x() - width() * 0.5) / newScale - oldViewX);
            m_pan.setY(-(currentCenter.y() - height() * 0.5) / newScale - oldViewY);
            m_lastTouchCenter = currentCenter;
            m_lastTouchDistance = currentDistance;
        }

        bool HandleTouchEvent(QTouchEvent* event)
        {
            if (event == nullptr)
            {
                return false;
            }
            const QVector<QPointF> positions = ActiveTouchPositions(event);
            if (event->type() == QEvent::TouchBegin)
            {
                m_rotating = false;
                m_panning = false;
                m_leftDragActive = false;
                unsetCursor();
                if (positions.size() >= 2)
                {
                    BeginTwoFingerTouch(positions.at(0), positions.at(1));
                }
                else if (positions.size() == 1)
                {
                    BeginSingleTouch(positions.first());
                }
                event->accept();
                return true;
            }

            if (event->type() == QEvent::TouchCancel)
            {
                ResetTouchState();
                event->accept();
                return true;
            }

            if (event->type() == QEvent::TouchUpdate)
            {
                if (positions.size() >= 2)
                {
                    if (m_touchMode != TouchMode::TwoFinger)
                    {
                        BeginTwoFingerTouch(positions.at(0), positions.at(1));
                    }
                    else
                    {
                        PanAndZoomFromTouch(TouchCenter(positions.at(0), positions.at(1)), TouchDistance(positions.at(0), positions.at(1)));
                        update();
                    }
                    event->accept();
                    return true;
                }
                if (positions.size() == 1)
                {
                    const QPointF pos = positions.first();
                    if (m_touchMode != TouchMode::Single)
                    {
                        BeginSingleTouch(pos, m_touchMode != TouchMode::TwoFinger);
                    }
                    else
                    {
                        if (!m_touchSingleDragActive)
                        {
                            const QPointF totalDelta = pos - m_touchStartPos;
                            if (std::hypot(totalDelta.x(), totalDelta.y()) < QApplication::startDragDistance())
                            {
                                m_lastTouchPos = pos;
                                event->accept();
                                return true;
                            }
                            m_touchSingleDragActive = true;
                        }
                        RotateByScreenDelta(pos - m_lastTouchPos);
                        m_lastTouchPos = pos;
                        update();
                    }
                    event->accept();
                    return true;
                }
            }

            if (event->type() == QEvent::TouchEnd)
            {
                if (m_touchMode == TouchMode::Single && m_allowTouchTapSelection && !m_touchSingleDragActive)
                {
                    SelectRotationCenterAt(m_lastTouchPos, 18.0);
                }
                if (positions.size() >= 2)
                {
                    BeginTwoFingerTouch(positions.at(0), positions.at(1));
                }
                else if (positions.size() == 1)
                {
                    BeginSingleTouch(positions.first(), false);
                }
                else
                {
                    ResetTouchState();
                }
                event->accept();
                return true;
            }

            event->accept();
            return true;
        }

        QVector<Layer> m_layers;
        QVector<DirectionArrow> m_arrows;
        PointCloudVec3 m_center;
        PointCloudVec3 m_selectedRotationCenter;
        double m_baseSpan = 200.0;
        double m_zoom = 1.0;
        QPointF m_pan;
        PointCloudVec3 m_right = { 1.0, 0.0, 0.0 };
        PointCloudVec3 m_up = { 0.0, 1.0, 0.0 };
        PointCloudVec3 m_forward = { 0.0, 0.0, 1.0 };
        QPointF m_lastMousePos;
        QPointF m_pressMousePos;
        bool m_rotating = false;
        bool m_panning = false;
        bool m_leftDragActive = false;
        int m_selectedLayerIndex = -1;
        int m_selectedPointIndex = -1;
        enum class TouchMode
        {
            None,
            Single,
            TwoFinger
        };
        TouchMode m_touchMode = TouchMode::None;
        QPointF m_touchStartPos;
        QPointF m_lastTouchPos;
        QPointF m_lastTouchCenter;
        double m_lastTouchDistance = 0.0;
        bool m_touchSingleDragActive = false;
        bool m_allowTouchTapSelection = false;
    };
} // namespace pcview
