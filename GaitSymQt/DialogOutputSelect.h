#ifndef DIALOGOUTPUTSELECT_H
#define DIALOGOUTPUTSELECT_H

#include "Preferences.h"

#include <QDialog>
#include <QFont>

class QListWidget;
class QLabel;
class QDialogButtonBox;
class QGridLayout;

namespace Ui {
    class DialogOutputSelect;
}

class DialogOutputSelect : public QDialog {
    Q_OBJECT
public:
    DialogOutputSelect(QWidget *parent = 0);
    ~DialogOutputSelect();

    Ui::DialogOutputSelect *ui() { return m_ui; }

    QListWidget *listWidgetBody;
    QListWidget *listWidgetDataTarget;
    QListWidget *listWidgetDriver;
    QListWidget *listWidgetGeom;
    QListWidget *listWidgetJoint;
    QListWidget *listWidgetMuscle;
    QListWidget *listWidgetReporter;
    QListWidget *listWidgetWarehouse;


    Preferences preferences() const;
    void setPreferences(const Preferences &preferences);

public slots:
    void menuRequestMuscle(QPoint);
    void menuRequestBody(QPoint);
    void menuRequestJoint(QPoint);
    void menuRequestGeom(QPoint);
    void menuRequestDriver(QPoint);
    void menuRequestDataTarget(QPoint);
    void menuRequestReporter(QPoint);
    void menuRequestWarehouse(QPoint);
    void acceptButtonClicked();
    void rejectButtonClicked();

protected:
    void changeEvent(QEvent *e);

    QGridLayout *gridLayout;
    QDialogButtonBox *buttonBox;

    int m_columns;
    Preferences m_preferences;

private:

    Ui::DialogOutputSelect *m_ui;
};

#endif // DIALOGOUTPUTSELECT_H
