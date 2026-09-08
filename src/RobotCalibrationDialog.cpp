#include "RobotCalibrationDialog.h"
#include "RobotCalibrationService.h"
#include "RobotDriverAdaptor.h"
#include "RobotDataHelper.h"
#include "RobotOperationLease.h"
#include "ConfigDatabase.h"
#include "HandEyeMatrixConfig.h"
#include "WindowStyleHelper.h"
#include <QComboBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include <QJsonDocument>
#include <QJsonArray>
#include <future>
#include <mutex>

namespace
{
struct WorkResult {RobotCalibrationRunResult run;RobotControllerHandEye eye;bool eyeRead=false;};
class CalibrationDialog final:public QDialog
{
public:
    CalibrationDialog(ContralUnit* units,int selected,QWidget* parent):QDialog(parent),units_(units)
    {
        setWindowTitle("标定资产与运动学模型优化");ApplyUnifiedWindowChrome(this);resize(1040,760);
        auto* layout=new QVBoxLayout(this);auto* scroll=new QScrollArea(this);scroll->setWidgetResizable(true);
        auto* page=new QWidget;auto* body=new QVBoxLayout(page);scroll->setWidget(page);layout->addWidget(scroll);
        auto* hint=new QLabel("品牌通用：适配接口 → 固定来源发现 → 身份/坐标校验 → 数据库记录。\n模型优化仅调用控制器正逆解计算，不移动、不上电、不切模式；优化候选不自动替换生产模型。",page);
        hint->setWordWrap(true);body->addWidget(hint);
        auto* form=new QFormLayout;form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);body->addLayout(form);
        robots_=new QComboBox;for(const auto& r:RobotDataHelper::LoadRobotList(units))robots_->addItem(r.displayName,r.unitIndex);
        const int selectedIndex=robots_->findData(selected);if(selectedIndex>=0)robots_->setCurrentIndex(selectedIndex);
        form->addRow("机器人",robots_);cameras_=new QComboBox;form->addRow("导入到相机分组",cameras_);
        sensor_=Spin(0,15,0);tool_=Spin(0,15,1);work_=Spin(0,15,1);load_=Spin(0,15,0);
        training_=Spin(80,1000,240);validation_=Spin(40,300,80);
        form->addRow("控制器激光传感器编号",sensor_);form->addRow("模型计算工具号",tool_);
        form->addRow("模型计算工件号",work_);form->addRow("模型计算附加载荷号",load_);
        form->addRow("训练样本数",training_);form->addRow("独立验证样本数",validation_);
        tolerance_=new QDoubleSpinBox;tolerance_->setRange(.001,.1);tolerance_->setDecimals(3);tolerance_->setValue(.05);tolerance_->setSuffix(" mm");
        form->addRow("优化验收位置上限（采样前固定）",tolerance_);
        discover_=new QPushButton("只读搜索/获取标定资产");optimize_=new QPushButton("开始运动学模型优化（不运动）");
        readEye_=new QPushButton("读取控制器相机矩阵");importEye_=new QPushButton("核对后导入当前相机分组");importEye_->setEnabled(false);
        auto* row=new QGridLayout;row->addWidget(discover_,0,0);row->addWidget(optimize_,0,1);row->addWidget(readEye_,1,0);row->addWidget(importEye_,1,1);body->addLayout(row);
        progress_=new QLabel;progress_->setWordWrap(true);body->addWidget(progress_);
        history_=new QComboBox;body->addWidget(history_);
        report_=new QPlainTextEdit;report_->setReadOnly(true);report_->setMinimumHeight(230);body->addWidget(report_,1);
        auto* footer=new QHBoxLayout;cancel_=new QPushButton("取消后台操作");cancel_->setEnabled(false);
        auto* close=new QPushButton("关闭");footer->addWidget(cancel_);footer->addStretch();footer->addWidget(close);layout->addLayout(footer);
        connect(close,&QPushButton::clicked,this,&QDialog::reject);
        connect(cancel_,&QPushButton::clicked,this,[this]{cancelFlag_.store(true);progress_->setText("正在取消，保留已完成的采样和证据；没有发送机器人停止命令。");});
        connect(robots_,qOverload<int>(&QComboBox::currentIndexChanged),this,[this]{RefreshRobot();});
        connect(history_,qOverload<int>(&QComboBox::currentIndexChanged),this,[this]{ShowHistory();});
        connect(sensor_,qOverload<int>(&QSpinBox::valueChanged),this,[this]{eyeValid_=false;importEye_->setEnabled(false);});
        connect(cameras_,qOverload<int>(&QComboBox::currentIndexChanged),this,[this]{importEye_->setEnabled(eyeValid_);});
        connect(discover_,&QPushButton::clicked,this,[this]{Start(0);});
        connect(optimize_,&QPushButton::clicked,this,[this]{Start(1);});
        connect(readEye_,&QPushButton::clicked,this,[this]{Start(2);});
        connect(importEye_,&QPushButton::clicked,this,[this]{Import();});
        auto* timer=new QTimer(this);timer->setInterval(200);connect(timer,&QTimer::timeout,this,[this]{Poll();});timer->start();RefreshRobot();
    }
    ~CalibrationDialog() override {cancelFlag_.store(true);if(future_.valid())future_.wait();}
    void reject() override {if(Busy()){cancelFlag_.store(true);progress_->setText("等待后台取消完成后可关闭。");return;}QDialog::reject();}
protected:
    void closeEvent(QCloseEvent* event) override {if(Busy()){cancelFlag_.store(true);event->ignore();return;}QDialog::closeEvent(event);}
private:
    bool Busy()const{return future_.valid();}
    static QSpinBox* Spin(int lo,int hi,int value){auto* s=new QSpinBox;s->setRange(lo,hi);s->setValue(value);return s;}
    RobotDriverAdaptor* Driver()const {return RobotDataHelper::GetRobotDriver(units_,robots_->currentData().toInt());}
    void RefreshRobot()
    {
        eyeValid_=false;cameras_->clear();auto* d=Driver();if(!d)return;
        const auto robot=QString::fromStdString(d->RobotName());const auto selected=RobotDataHelper::MeasureCameraSection(robot);
        for(const auto& c:RobotDataHelper::LoadCameraList(robot))cameras_->addItem(c.displayName,c.sectionName);
        const auto index=cameras_->findData(selected);if(index>=0)cameras_->setCurrentIndex(index);
        RefreshHistory();SetBusy(false);
    }
    void RefreshHistory()
    {
        history_->clear();auto* d=Driver();if(!d)return;
        QMap<QString,QMap<QString,QString>> snapshots;QString error;
        if(ConfigDatabase::ReadScopedModuleSnapshot("robot",QString::fromStdString(d->RobotName()),"CalibrationRuns",snapshots,&error))
            for(auto it=snapshots.constEnd();it!=snapshots.constBegin();)
            {
                --it;
                if(it.value().contains("Report"))history_->addItem(it.key(),it.value()["Report"]);
            }
        if(history_->count()==0)report_->setPlainText(RobotCalibrationService::LatestReport(d->RobotName()));else ShowHistory();
    }
    void ShowHistory(){if(history_->currentIndex()>=0)report_->setPlainText(history_->currentData().toString());}
    void SetBusy(bool busy)
    {
        for(QWidget* w:std::vector<QWidget*>{robots_,cameras_,history_,sensor_,tool_,work_,load_,training_,validation_,tolerance_,discover_,readEye_})w->setEnabled(!busy);
        auto* d=Driver();optimize_->setEnabled(!busy&&d&&d->Supports(RobotDriverCapability::ControllerKinematicsCalculate));
        optimize_->setToolTip(d&&d->Supports(RobotDriverCapability::ControllerKinematicsCalculate)?"仅计算，不运动":"品牌底层尚未接入控制器正逆解；禁止以本地结果自证。");
        importEye_->setEnabled(!busy&&eyeValid_);cancel_->setEnabled(busy);
    }
    void Start(int kind)
    {
        if(Busy())return;auto* driver=Driver();if(!driver||!driver->IsConnected()){QMessageBox::warning(this,"标定资产","请先完成所选机器人的连接测试。");return;}
        cancelFlag_.store(false);eyeValid_=false;SetBusy(true);
        RobotKinematicsOptimizationOptions options;options.profile={tool_->value(),work_->value(),load_->value()};
        options.trainingCount=training_->value();options.validationCount=validation_->value();options.maxPositionErrorMm=tolerance_->value();
        const int sensor=sensor_->value();
        future_=std::async(std::launch::async,[this,driver,kind,options,sensor]{
            WorkResult out;auto progress=[this](const std::string& text){std::lock_guard<std::mutex> lock(progressMutex_);progressText_=QString::fromStdString(text);};
            if(kind==0)out.run=driver->AcquireCalibrationAssets(cancelFlag_,progress);
            else if(kind==1)out.run=driver->OptimizeKinematicsModel(options,cancelFlag_,progress);
            else
            {
                QString why;auto lease=RobotOperationLease::TryAcquire(driver,"读取控制器手眼标定",&why);
                if(!lease){out.run.report=why.toStdString();return out;}
                std::string error;out.eyeRead=driver->ReadControllerHandEye(sensor,out.eye,error);out.run.passed=out.eyeRead;
                out.run.report=out.eyeRead?out.eye.source+"\n相机="+out.eye.cameraAddress+"，绑定Tool="+std::to_string(out.eye.toolIndex)
                    +"\n"+out.eye.evidence:error;
            }
            return out;
        });
    }
    void Poll()
    {
        if(!Busy())return;
        {std::lock_guard<std::mutex> lock(progressMutex_);if(!progressText_.isEmpty())progress_->setText(progressText_);}
        if(future_.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready)return;
        WorkResult out;try{out=future_.get();}catch(const std::exception& e){out.run.report=e.what();}
        if(out.eyeRead&&!cancelFlag_.load()){eye_=out.eye;eyeValid_=true;}
        RefreshHistory();report_->setPlainText(QString::fromStdString(out.run.report));
        progress_->setText(out.run.passed?"已完成并核对；详见证据。":"未通过/受限/已取消，详见证据。");SetBusy(false);
    }
    void Import()
    {
        if(Busy()||!eyeValid_)return;auto* driver=Driver();if(!driver)return;
        const auto robot=QString::fromStdString(driver->RobotName()),camera=cameras_->currentData().toString();
        RobotDataHelper::CameraParamData parameters;QString why;
        if(camera.isEmpty()||!RobotDataHelper::LoadCameraParam(robot,camera,parameters,&why)
            ||parameters.deviceAddress.trimmed()!=QString::fromStdString(eye_.cameraAddress))
        {QMessageBox::warning(this,"相机归属不匹配","控制器标定相机IP与所选数据库相机IP不一致，禁止导入。\n控制器相机="+QString::fromStdString(eye_.cameraAddress));return;}
        if(QMessageBox::question(this,"导入控制器手眼矩阵",
            QString("来源：%1\n导入到：%2 / %3\n参考系：相机 → Tool%4 TCP\n将备份旧矩阵和来源，再替换本机数据库记录；不改控制器。导入后必须执行流程7实测。是否确认？")
            .arg(QString::fromStdString(eye_.source),robot,camera).arg(eye_.toolIndex),QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes)return;
        // A short second read occurs in the background before committing: UI
        // confirmation cannot authorize a different controller/profile snapshot.
        cancelFlag_.store(false);SetBusy(true);const auto expected=eye_;
        future_=std::async(std::launch::async,[this,driver,robot,camera,expected]{
            WorkResult out;QString why;auto lease=RobotOperationLease::TryAcquire(driver,"核对并导入控制器手眼",&why);
            if(!lease){out.run.report=why.toStdString();return out;}std::string error;
            if(!driver->ValidateControllerHandEyeContext(expected,error)){out.run.report=error;return out;}
            if(cancelFlag_.load()){out.run.report="已取消导入。";return out;}
            const auto location=GetHandEyeMatrixLocation(robot,camera);
            QMap<QString,QMap<QString,QString>> old;
            if(!ConfigDatabase::ReadScopedModuleSnapshot(location.scopeType,location.scopeId,location.module,old,&why))
            {out.run.report="旧矩阵备份读取失败："+why.toStdString();return out;}
            const QString backup="CalibrationBackups/"+QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz");
            if(!ConfigDatabase::ReplaceScopedModuleSectionsAtomically("robot",robot,backup,old,{},&why))
            {out.run.report="备份旧矩阵失败："+why.toStdString();return out;}
            QMap<QString,QMap<QString,QString>> values;
            values["Base"]={{"Version","1"},{"RobotName",robot},{"CameraSection",camera},{"RobotType",QString::number(driver->DriverDescriptor().poseConventionType)},
                {"Calibrated","1"},{"ControllerSensor",QString::number(expected.sensorIndex)},{"ControllerTool",QString::number(expected.toolIndex)},
                {"ControllerIdentity",QString::fromStdString(expected.controllerIdentity)},{"ControllerSource",QString::fromStdString(expected.source)},
                {"ControllerSourceSha256",QString::fromStdString(expected.sourceFingerprint)},{"ControllerToolSha256",QString::fromStdString(expected.toolFingerprint)},
                {"ControllerCameraIP",QString::fromStdString(expected.cameraAddress)},{"ControllerFrame","camera-to-tool-tcp"},{"ImportEvidence",QString::fromStdString(expected.evidence)}};
            for(int r=0;r<3;++r){for(int c=0;c<3;++c)values["HandEyeMatrix"][QString("R%1%2").arg(r).arg(c)]=QString::number(expected.cameraToTool(r,c),'g',17);
                values["HandEyeMatrix"][QString("T%1").arg(r)]=QString::number(expected.cameraToTool(r,3),'g',17);}
            if(!ConfigDatabase::ReplaceScopedModuleSectionsAtomically(location.scopeType,location.scopeId,location.module,values,{},&why))
            {out.run.report="导入矩阵失败："+why.toStdString();return out;}
            const bool ready=ConfigDatabase::WriteScopedSetting("robot",robot,"RobotPara/SetupStatus","HandEyeReady","1");
            out.run.passed=ready;out.run.report=ready?"已导入并保存相机/工具/控制器/标定源绑定，旧记录已备份。请执行流程7实测验证。":"矩阵已保存，但就绪状态保存失败，需检查数据库。";
            return out;
        });
    }
    ContralUnit* units_;QComboBox *robots_,*cameras_,*history_;QSpinBox *sensor_,*tool_,*work_,*load_,*training_,*validation_;
    QDoubleSpinBox* tolerance_;QPushButton *discover_,*optimize_,*readEye_,*importEye_,*cancel_;QLabel* progress_;QPlainTextEdit* report_;
    std::future<WorkResult> future_;std::atomic_bool cancelFlag_{false};std::mutex progressMutex_;QString progressText_;bool eyeValid_=false;RobotControllerHandEye eye_;
};
}
void OpenRobotCalibrationDialog(ContralUnit* units,int selectedUnit,QWidget* parent)
{CalibrationDialog dialog(units,selectedUnit,parent);dialog.exec();}
