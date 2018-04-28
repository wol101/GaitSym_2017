#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileInfo>

namespace Ui
{
    class MainWindow;
}

class GLWidget;
class DialogVisibility;
class QBoxLayout;
class QListWidgetItem;
class ViewControlWidget;
class DialogLog;
class Preferences;
class SimulationWindow;
class Simulation;
class AVIWriter;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    Preferences *GetPreferences() { return m_preferences; }
    Simulation *GetSimulation() { return m_simulation; }
    SimulationWindow *GetSimulationWindow() { return m_simulationWindow; }
    long long GetStepCount() { return m_stepCount; }

public slots:
    void setInterfaceValues();
    void processOneThing();

    void about();
    void buttonCameraFront();
    void buttonCameraRight();
    void buttonCameraTop();
    void buttonCameraBack();
    void buttonCameraLeft();
    void buttonCameraBottom();
    void checkboxActivationColours(int);
    void checkboxContactForce(int);
    void checkboxMuscleForce(int);
    void checkboxTracking(int);
    void listBodyChecked(QListWidgetItem*);
    void listGeomChecked(QListWidgetItem*);
    void listJointChecked(QListWidgetItem*);
    void listMuscleChecked(QListWidgetItem*);
    void menu1280x720();
    void menu1920x1080();
    void menu640x480();
    void menu800x600();
    void menuDefaultView();
    void menuImportWarehouse();
    void menuOutputs();
    void menuPreferences();
    void menuRecordMovie();
    void menuRequestBody(QPoint);
    void menuRequestGeom(QPoint);
    void menuRequestJoint(QPoint);
    void menuRequestMuscle(QPoint);
    void menuSaveDefaultView();
    void menuStartAVISave();
    void menuStartOBJSequenceSave();
    void menuStartWarehouseExport();
    void menuStopAVISave();
    void menuStopOBJSequenceSave();
    void menuStopWarehouseExport();
    void menuToggleFullScreen();
    void menuGeneticAlgorithm();
    void menuNextAscentHillclimbing();
    void menuRandomAscentHillclimbing();
    void menuSimplexSearch();
    void menuSimulatedAnnealing();
    void menuTabuSearch();
    void newFromGeometry();
    void open();
    void resizeAndCentre(int w, int h);
    void restart();
    void run();
    void saveas();
    void saveasworld();
    void snapshot();
    void objSnapshot();
    void spinboxCOIXChanged(double);
    void spinboxCOIYChanged(double);
    void spinboxCOIZChanged(double);
    void spinboxCursorNudgeChanged(double);
    void spinboxCursorSizeChanged(double);
    void spinboxDistanceChanged(double);
    void spinboxFarChanged(double);
    void spinboxFoVChanged(double);
    void spinboxFPSChanged(double);
    void spinboxNearChanged(double);
    void spinboxSkip(int);
    void spinboxTimeMax(double);
    void spinboxTrackingOffsetChanged(double v);
    void step();

    void copy();
    void cut();
    void paste();
    void selectAll();

    void setStatusString(QString s);
    void setUICOI(double x, double y, double z);
    void setUIFoV(double v);

    void log(const char *text);

protected:
    void closeEvent(QCloseEvent *event) Q_DECL_OVERRIDE;

private:

    void open(const QString &fileName);
    void fillVisibitilityLists();
    void writeSettings();
    static QByteArray readResource(const QString &resource);

    Ui::MainWindow *ui;

    QFileInfo configFile;

    bool m_movieFlag;
    bool m_saveOBJFileSequenceFlag;
    QString m_objFileSequenceFolder;
    bool m_newFromGeometryFlag;
    bool m_stepFlag;
    long long m_stepCount;

    Preferences *m_preferences;
    QTimer *m_timer;
    SimulationWindow *m_simulationWindow;
    Simulation *m_simulation;
};

#endif // MAINWINDOW_H
