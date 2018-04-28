#ifndef VIEWCONTROLWIDGET_H
#define VIEWCONTROLWIDGET_H

#include <QWidget>

class QPixmap;

class ViewControlWidget : public QWidget
{
    Q_OBJECT

public:
    ViewControlWidget(QWidget *parent = 0);
    ~ViewControlWidget();

    int FindClosestVertex(double data[][3], int count, double x, double y, double z);

public slots:

signals:
    void EmitCameraVec(double x, double y, double z);

protected:
    void mousePressEvent(QMouseEvent *event);
    void paintEvent (QPaintEvent *);

private:
    QPixmap *backgroundImage;
    QPixmap *blob;

    double lastX;
    double lastY;
    double lastZ;
};

#endif // VIEWCONTROLWIDGET_H
