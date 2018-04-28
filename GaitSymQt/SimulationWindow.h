#ifndef SIMULATIONWINDOW_H
#define SIMULATIONWINDOW_H

#include "IrrlichtWindow.h"

class Simulation;
class FacetedObject;
class Trackball;
class AVIWriter;

class SimulationWindow : public IrrlichtWindow
{
    Q_OBJECT
public:
    SimulationWindow(QWindow *parent = 0);
    ~SimulationWindow();

    void initialiseScene();

    void updateCamera();

    irr::scene::ISceneNode* getSceneNodeAndCollisionPointFromRawScreenCoordinates(int screenX, int screenY, irr::s32 idBitMask,
                                                                                  irr::core::vector3df &outCollisionPoint, irr::core::triangle3df &outTriangle);


    Simulation *simulation() const;
    void setSimulation(Simulation *simulation);

    bool wireFrame() const;
    void setWireFrame(bool wireFrame);

    bool boundingBox() const;
    void setBoundingBox(bool boundingBox);

    bool orthographicProjection() const;
    void setOrthographicProjection(bool orthographicProjection);

    float cameraDistance() const;
    void setCameraDistance(float cameraDistance);

    float FOV() const;
    void setFOV(float FOV);

    float cameraVecX() const;
    void setCameraVecX(float cameraVecX);

    float cameraVecY() const;
    void setCameraVecY(float cameraVecY);

    float cameraVecZ() const;
    void setCameraVecZ(float cameraVecZ);

    float COIx() const;
    void setCOIx(float COIx);

    float COIy() const;
    void setCOIy(float COIy);

    float COIz() const;
    void setCOIz(float COIz);

    float frontClip() const;
    void setFrontClip(float frontClip);

    float backClip() const;
    void setBackClip(float backClip);

    float upX() const;
    void setUpX(float upX);

    float upY() const;
    void setUpY(float upY);

    float upZ() const;
    void setUpZ(float upZ);

    float cursorRadius() const;
    void setCursorRadius(float cursorRadius);

    float cursor3DNudge() const;
    void setCursor3DNudge(float cursor3DNudge);

    irr::video::SColor cursorColour() const;
    void setCursorColour(const irr::video::SColor &cursorColour);

    bool normals() const;
    void setNormals(bool normals);

    bool halfTransparency() const;
    void setHalfTransparency(bool halfTransparency);

    irr::scene::ISceneNode *highlightedNode() const;
    void setHighlightedNode(irr::scene::ISceneNode *highlightedNode);

    int WriteStillFrame(const QString &filename);
    int WriteMovieFrame();
    int WriteCADFrame(const QString &pathname);
    int StartAVISave(const QString &fileName);
    int StopAVISave();


    void GetAllChildren(irr::scene::ISceneNode *sceneNode, irr::core::list<irr::scene::ISceneNode *> *sceneNodeList);

    // utilities
    void SetMeshColour(irr::scene::IMesh *mesh, const irr::video::SColor &colour);

    AVIWriter *aviWriter() const;
    void setAviWriter(AVIWriter *aviWriter);

    int aviQuality() const;
    void setAviQuality(int aviQuality);

public slots:
    void SetCameraVec(double x, double y, double z);

signals:
    void EmitStatusString(QString s);
    void EmitCOI(double x, double y, double z);
    void EmitFoV(double v);

protected:
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void wheelEvent(QWheelEvent * event) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

private:
    Simulation *m_simulation;

    bool m_wireFrame;
    bool m_boundingBox;
    bool m_boundingBoxBuffers;
    bool m_normals;
    bool m_halfTransparency;
    bool m_orthographicProjection;

    float m_cameraDistance;
    float m_FOV;
    float m_cameraVecX, m_cameraVecY, m_cameraVecZ;
    float m_COIx, m_COIy, m_COIz;
    float m_frontClip;
    float m_backClip;
    float m_upX, m_upY, m_upZ;

    irr::core::matrix4 m_projectionMatrix;
    irr::core::matrix4 m_viewMatrix;

    Trackball *m_trackball;
    bool m_trackballFlag;
    irr::core::vector3df m_trackballStartCameraVec;
    irr::core::vector3df m_trackballStartUp;

    bool m_panFlag;
    irr::core::matrix4 m_projectPanMatrix;
    irr::core::matrix4 m_unprojectPanMatrix;
    irr::core::vector3df m_panStartPoint;
    irr::core::vector3df m_panStartCOI;
    irr::core::vector3df m_panStartScreenPoint;

    bool m_zoomFlag;
    double m_zoomDistance;
    double m_zoomStartFOV;

    irr::scene::ICameraSceneNode* m_camera;
    irr::scene::IMeshSceneNode* m_cursor3D;
    float m_cursorRadius;
    float m_cursor3DNudge;
    irr::video::SColor m_cursorColour;

    irr::scene::ISceneNode* m_highlightedNode;

    AVIWriter *m_aviWriter;
    int m_aviQuality;
    unsigned int m_fps;
};

#endif // SIMULATIONWINDOW_H
