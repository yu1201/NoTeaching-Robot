#include "CliHelp.h"

#include <QTextStream>

void WriteCliHelp(QTextStream& out)
{
    out << "QtWidgetsApplication4 command line options:\n";
    out << "  --no-show                         不显示主窗口，适合自动测试\n";
    out << "  --data-root <DIR>                 指定 Data/Result/Log/Temp/Job 的统一可写根目录\n";
    out << "  --print-app-paths-json            输出路径诊断 JSON 后退出（自动化验收用）\n";
    out << "  --open-function-test              打开机器人功能测试窗口\n";
    out << "  --open-jog                        打开机器人点动控制窗口\n";
    out << "  --open-precise-measure            打开测量焊接参数窗口\n";
    out << "  --open-camera-param               打开相机参数窗口\n";
    out << "  --robot <UnitNo|RobotA|RobotB|中文名> 选择通用机器人CLI目标，默认当前/第一个可用机器人\n";
    out << "  --robot-connect                   连接选中的机器人驱动\n";
    out << "  --robot-movel <X,Y,Z,RX,RY,RZ[,BX,BY,BZ]> 发送直角 MOVL，默认速度500mm/min\n";
    out << "  --robot-movel-relative <DX,DY,DZ[,DRX,DRY,DRZ,BX,BY,BZ]> 基于当前位置做直角相对MOVL\n";
    out << "  --robot-movj <S,L,U,R,B,T[,EX1,EX2,EX3]> 发送关节脉冲 MOVJ，默认速度1%\n";
    out << "  --robot-speed <VALUE>             覆盖本次运动速度，MOVL按mm/min，MOVJ按驱动百分比/约定\n";
    out << "  --robot-done-delay <ms>           运动完成轮询间隔，默认200ms\n";
    out << "  --robot-no-wait                   运动下发后不等待完成\n";
    out << "  --fanuc-connect                   连接 FANUC 常驻服务端口\n";
    out << "  --fanuc-upload-services           上传/编译 FANUC 服务库和固定 TP\n";
    out << "  --skip-upload-wait                上传服务后不等待回车，自动化测试用\n";
    out << "  --fanuc-curpos-diag               运行当前位置/PR20 诊断命令\n";
    out << "  --fanuc-pr20-diag                 仅读取 FANUC PR[20] 诊断点\n";
    out << "  --fanuc-raw <CMD>                 发送一条原始 FANUC 服务命令\n";
    out << "  --fanuc-call <PROGRAM>            调用机器人程序\n";
    out << "  --measure-then-weld-scan-only-repeat <N> 自动执行先测后焊扫描流程N次，仅到收枪安全位置，不执行焊接，目标机器人同--robot\n";
    out << "  --measure-then-weld-scan-speed <mm/min> 覆盖本次CLI先测后焊扫描速度，不修改配置库\n";
    out << "  --measure-then-weld-camera-offset-ms <ms> 覆盖本次CLI相机时间补偿，不修改配置库\n";
    out << "  --laser-classify <FILE>           对激光点云做去噪/拟合/起终点拐点分类\n";
    out << "  --laser-classify-dir <DIR>        批量处理目录下所有 PreciseLaserPoint.txt\n";
    out << "  --laser-classify-output <FILE>    指定分类结果输出文件\n";
    out << "  --rebuild-measure-weld-files <DIR> 从 LaserPoint 目录重建 PreservePath、焊接姿态和补偿文件，参数机器人同--robot\n";
    out << "  --pointcloud-processing-mode <sdk|sdkfit|cloudfit|legacy> 仅本次CLI覆盖点云处理方式，不写入配置库\n";
    out << "  --pointcloud-scan-direction <X,Y,Z> 仅本次CLI覆盖点云SDK扫描方向，用于离线测试\n";
    out << "  --apply-weld-seam-comp <FILE>     对焊道姿态文件应用配置库中的焊道补偿\n";
    out << "  --apply-weld-seam-comp-output <FILE> 指定补偿结果输出文件，默认另存 _SeamComp\n";
    out << "  --generate-step-weld-program <FILE> 根据焊接姿态文件生成 STEP Weld_时间.srp/.srd，默认按实际焊接生成ARCON/ARCOFF\n";
    out << "  --generate-step-weld-program-output-dir <DIR> 指定 STEP 焊接程序输出目录，默认 Job\\STEP\n";
    out << "  --generate-step-weld-program-dry-run 按空跑轨迹生成 STEP 文件，不生成ARCON/ARCSET/ARCOFF焊接指令\n";
    out << "  --test-pointwise-weave [shape amp freq] 离线测试pointwise摆动：造直线中心线跑摆动算法，输出 WeaveTest_centerline/weave.txt（默认 5 3 2，不连机器人）\n";
    out << "  --generate-step-weld-speed <mm/min> 覆盖本次 STEP 文件轨迹速度，不修改配置库\n";
    out << "  --update-weld-pose-average <FILE_OR_DIR> 离线统计四类焊道平均姿态并更新补偿姿态库\n";
    out << "  --quit-after <ms>                 指定毫秒后退出程序\n";
}
