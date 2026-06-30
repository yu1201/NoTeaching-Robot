// 自包含端到端回归测试：合成一段波纹板基础焊道，跑 SdkBaseWeldFit 等效管线，断言拐点结构不变量
// （内/外角严格交替、主轴单调、无平台塌陷长段）；另测双边预平滑「去锯齿 + 保台阶」。无外部数据依赖。
// 返回 0=全部通过，非 0=有失败。CMake 里注册为 ctest 测试。
#include "MeasureThenWeldFilterFit.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace mtw_filter_fit;

namespace
{
int g_failures = 0;
void Check(bool ok, const char* what)
{
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_failures;
}

// 合成波纹：沿主轴 Y=s，侧向 X=h。低平台(-hi)→上坡→高平台(+hi)→下坡，重复 periods 次。
std::vector<IndexedPoint3D> MakeCorrugated(int periods, double plat, double edge, double hi, double step)
{
    std::vector<IndexedPoint3D> pts;
    const double period = 2.0 * plat + 2.0 * edge;
    const double total = periods * period;
    int idx = 0;
    for (double s = 0.0; s <= total + 1e-9; s += step)
    {
        const double u = std::fmod(s, period);
        double h;
        if (u < plat) h = -hi;                                   // 低平台
        else if (u < plat + edge) h = -hi + (u - plat) / edge * 2.0 * hi;        // 上坡
        else if (u < 2.0 * plat + edge) h = hi;                  // 高平台
        else h = hi - (u - 2.0 * plat - edge) / edge * 2.0 * hi; // 下坡
        IndexedPoint3D p;
        p.index = idx++;
        p.point = Point3D{ h, s, 0.0 };
        pts.push_back(p);
    }
    return pts;
}

double TotalVariationX(const std::vector<IndexedPoint3D>& pts)
{
    double tv = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) tv += std::abs(pts[i].point.x - pts[i - 1].point.x);
    return tv;
}
}

int main()
{
    std::printf("== MeasureThenWeldFilterFit regression ==\n");

    // ---- 1) 波纹拐点管线不变量 ----
    const int periods = 6;
    const double plat = 40.0, edge = 35.0, hi = 15.0, period = 2.0 * plat + 2.0 * edge;
    const std::vector<IndexedPoint3D> wave = MakeCorrugated(periods, plat, edge, hi, 0.5);

    FilterFitParams pr = MeasureThenWeldDefaultParams(SampleAxis::AxisY);
    pr.inputAlreadyDenoised = true;          // SdkBase 路径：方位角法 + 跳过自身去噪
    pr.cornerPatternRefitEnable = true;      // 平台重算
    pr.enableEndPeriodCornerRecover = true;  // 端区周期补漏 + 删错
    pr.enablePlatformCornerSnap = true;      // 按平台边界重定

    const AnalysisResult a = AnalyzeMeasureThenWeldPath(wave, pr);
    std::printf("波纹: 输入=%zu  关键点=%zu  (周期=%g, 期数=%d)\n", wave.size(), a.keyPoints.size(), period, periods);
    Check(a.ok, "analyze ok");
    Check(a.keyPoints.size() >= static_cast<std::size_t>(3 * periods), "corner density >= 3/period");
    if (a.keyPoints.size() >= 3)
    {
        Check(a.keyPoints.front().type == WeldPointType::Start, "first is Start");
        Check(a.keyPoints.back().type == WeldPointType::End, "last is End");

        bool monotonic = true, noCollapse = true;
        double maxGap = 0.0;
        for (std::size_t i = 1; i < a.keyPoints.size(); ++i)
        {
            const double dy = a.keyPoints[i].point.y - a.keyPoints[i - 1].point.y;
            if (dy <= 0.0) monotonic = false;
            const double gap = std::abs(dy);
            if (i >= 2 && i + 1 < a.keyPoints.size() && gap > maxGap) maxGap = gap;  // 仅看中段相邻角间距
        }
        // 波纹结构：每个平台由两个【同类】边界角界定(高平台 II / 低平台 OO)，拐点序列呈 …II OO II OO…。
        // 正确不变量是「同类游程长度 ≤ 2」(出现 3 连续同类=漏拐点/平台合并)，而非严格交替。
        bool typesOk = true;
        int maxRun = 0, run = 0, innerCnt = 0, outerCnt = 0;
        WeldPointType last = WeldPointType::Start;
        bool havePrev = false;
        for (std::size_t i = 1; i + 1 < a.keyPoints.size(); ++i)
        {
            const WeldPointType t = a.keyPoints[i].type;
            if (t != WeldPointType::InnerCorner && t != WeldPointType::OuterCorner) { typesOk = false; break; }
            if (t == WeldPointType::InnerCorner) ++innerCnt; else ++outerCnt;
            run = (havePrev && t == last) ? run + 1 : 1;
            if (run > maxRun) maxRun = run;
            last = t; havePrev = true;
        }
        if (maxGap > 1.2 * period) noCollapse = false;  // 中段最大间距远超一个周期 = 平台塌陷/漏拐点
        std::printf("  中段最大相邻角间距=%.1f (阈值 %.1f)  同类最大游程=%d  内角=%d 外角=%d\n",
            maxGap, 1.2 * period, maxRun, innerCnt, outerCnt);
        Check(monotonic, "main-axis strictly increasing");
        Check(typesOk, "interior corners are inner/outer only");
        Check(maxRun <= 2, "same-type run <= 2 (II/OO platform-pair pattern)");
        Check(innerCnt > 0 && outerCnt > 0, "both inner and outer corners present");
        Check(noCollapse, "no platform collapse (max interior gap < 1.2*period)");
    }

    // ---- 2) 双边预平滑：去锯齿 ----
    std::vector<IndexedPoint3D> saw;
    for (int i = 0; i <= 200; ++i)
    {
        IndexedPoint3D p; p.index = i;
        p.point = Point3D{ (i % 2 == 0 ? 0.3 : -0.3), i * 0.5, 0.0 };  // 直线 + ±0.3 锯齿
        saw.push_back(p);
    }
    const double tvBefore = TotalVariationX(saw);
    const int moved = BilateralPresmoothSdkBaseWeld(saw, 3.0, 0.5);
    const double tvAfter = TotalVariationX(saw);
    std::printf("预平滑去锯齿: 移动点=%d  TV %.2f -> %.2f\n", moved, tvBefore, tvAfter);
    Check(moved > 0, "presmooth moved points");
    Check(tvAfter < 0.5 * tvBefore, "sawtooth total-variation reduced > 50%");

    // ---- 3) 双边预平滑：保住真实台阶 ----
    std::vector<IndexedPoint3D> stepData;
    for (int i = 0; i <= 200; ++i)
    {
        IndexedPoint3D p; p.index = i;
        p.point = Point3D{ (i < 100 ? 0.0 : 3.0), i * 0.5, 0.0 };  // 中点处 X 跳变 3mm 台阶
        stepData.push_back(p);
    }
    BilateralPresmoothSdkBaseWeld(stepData, 3.0, 0.5);
    const double stepKept = stepData[120].point.x - stepData[80].point.x;  // 跨台阶高度差
    std::printf("预平滑保台阶: 台阶高(原3.0) -> %.2f\n", stepKept);
    Check(stepKept > 2.5, "real step preserved (> 2.5 of 3.0)");

    std::printf("== %s (%d failures) ==\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
