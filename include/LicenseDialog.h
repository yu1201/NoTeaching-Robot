#pragma once

#include <QDialog>
#include <functional>

class QLabel;
class QLineEdit;
class QCloseEvent;

class LicenseDialog final : public QDialog
{
public:
    static QString ThemeStyleSheet();
    explicit LicenseDialog(QWidget* parent = nullptr,
        std::function<void()> safetyRecovery = {}, std::function<void()> safetyStop = {},
        std::function<bool()> safetyBusy = {}, bool acknowledgeBlockedGate = false);
    void reject() override;
protected:
    void closeEvent(QCloseEvent* event) override;
private:
    void Refresh();
    QLabel* title_ = nullptr;
    QLabel* status_ = nullptr;
    QLabel* details_ = nullptr;
    QLineEdit* code_ = nullptr;
    std::function<bool()> safetyBusy_;
};
