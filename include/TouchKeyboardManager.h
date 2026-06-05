#pragma once

#include <QObject>
#include <QPointer>
#include <QString>

class QEvent;
class QWidget;

class TouchKeyboardManager : public QObject
{
public:
    enum class Mode
    {
        Auto,
        Always,
        Never
    };

    explicit TouchKeyboardManager(QObject* parent = nullptr);

    void Install(QObject* target);
    void LoadSettings();
    void SaveSettings() const;
    void SetMode(Mode mode);
    Mode CurrentMode() const;

    static QString ModeToStorageString(Mode mode);
    static QString ModeDisplayName(Mode mode);
    static Mode ModeFromStorageString(const QString& value);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    bool ShouldHandleWidget(QWidget* widget) const;
    bool ShouldShowKeyboard() const;
    void MarkTouchInput();
    void MarkPhysicalKeyInput();
    void ScheduleKeyboardFor(QWidget* widget);
    void ShowKeyboard();
    void HideKeyboard();
    void EnsureKeyboardPanel();
    void PositionKeyboardPanel(QWidget* target);
    void SendKeyboardText(const QString& text);
    void SendKeyboardBackspace();
    void SendKeyboardClear();
    bool IsKeyboardWidget(QWidget* widget) const;
    bool IsNumericInputWidget(QWidget* widget) const;
    bool HasTouchCapability() const;
    bool HasHardwareKeyboard() const;

    QObject* m_pInstalledTarget;
    QPointer<QWidget> m_pKeyboardPanel;
    QPointer<QWidget> m_pKeyboardTarget;
    Mode m_mode;
    qint64 m_nStartMs;
    qint64 m_nLastTouchMs;
    qint64 m_nLastPhysicalKeyMs;
    qint64 m_nLastShowMs;
};
