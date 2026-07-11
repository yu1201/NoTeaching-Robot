#pragma once

#include <QDialog>
#include <QLabel>
#include <QMessageBox>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QVector>

#include <memory>

#include "Const.h"

class RobotDriverAdaptor;
class RobotOperationLease;
class QGridLayout;
class QCloseEvent;

class RobotJogDialog : public QDialog
{
public:
	explicit RobotJogDialog(RobotDriverAdaptor* robotDriver, QWidget* parent = nullptr);
	~RobotJogDialog() override;

protected:
	void closeEvent(QCloseEvent* event) override;

private:
	enum class JogMode
	{
		Cartesian,
		Joint
	};

	void BuildUi();
	void ApplyStyle();
	void AddAxisRow(QGridLayout* layout, int row, const QString& axisName, JogMode mode, int axisIndex, const QString& unitText);
	void LoadSpeedSettings();
	void SaveSpeedSettings() const;
	void ReadCurrentCartesianTarget();
	void ReadCurrentJointTarget();
	void MoveToCartesianTarget();
	void MoveToJointTarget();
	void StartJog(JogMode mode, int axisIndex, int direction);
	void BeginJog();
	void StepJog(JogMode mode, int axisIndex, int direction);
	void StopJog();
	void FeedNextPoint();
	void RefreshStateText();
	void UpdateMotionButtonState();
	void SetMotionTaskRunning(bool running);
	bool IsMotionBusy() const;
	double CartesianSpeed() const;
	double JointSpeed() const;
	T_ROBOT_COORS BuildCartesianStreamPoint(int stepIndex) const;
	T_ANGLE_PULSE BuildJointStreamPoint(int stepIndex) const;
	bool ReadCartesianTargetFromEditors(T_ROBOT_COORS& target, QString& error) const;
	bool ReadJointTargetFromEditors(T_ANGLE_PULSE& target, QString& error) const;
	void SetCartesianTargetEditors(const T_ROBOT_COORS& target);
	void SetJointTargetEditors(const T_ANGLE_PULSE& target);
	void ShowMessageOnUiThread(QMessageBox::Icon icon, const QString& title, const QString& text);

	RobotDriverAdaptor* m_robotDriver;
	QTimer* m_jogStartTimer;
	QTimer* m_jogTimer;
	QTimer* m_stateTimer;
	QLabel* m_stateLabel;
	QLineEdit* m_cartesianSpeedEdit;
	QLineEdit* m_jointSpeedEdit;
	QPushButton* m_stopButton;
	QVector<QPushButton*> m_motionButtons;
	QVector<QLineEdit*> m_cartesianTargetEdits;
	QVector<QLineEdit*> m_jointTargetEdits;
	bool m_jogActive;
	bool m_motionTaskRunning;
	std::shared_ptr<RobotOperationLease> m_jogOperationLease;
	JogMode m_currentMode;
	int m_currentAxis;
	int m_currentDirection;
	int m_nextStreamStep;
	double m_streamCartesianSpeed;
	double m_streamJointSpeed;
	T_ROBOT_COORS m_streamBasePos;
	T_ROBOT_COORS m_lastStreamPos;
	T_ANGLE_PULSE m_streamBasePulse;
	T_ANGLE_PULSE m_lastStreamPulse;
};
