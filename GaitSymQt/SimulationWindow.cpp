#include "SimulationWindow.h"
#include "Simulation.h"
#include "Body.h"
#include "FacetedObject.h"
#include "TrackBall.h"
#include "TIFFWrite.h"
#include "CCameraSceneNode.h"
#include "AVIWriter.h"

#include <QApplication>
#include <QClipboard>
#include <QWheelEvent>
#include <QElapsedTimer>
#include <QDir>
#include <QMessageBox>
#include <QDirIterator>
#include <QDebug>
#include <QBuffer>
#include <QtGlobal>

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

SimulationWindow::SimulationWindow(QWindow *parent)
    : IrrlichtWindow(parent)
{
    m_simulation = 0;
    m_wireFrame = false;
    m_boundingBox = false;
    m_boundingBoxBuffers = false;
    m_normals = false;
    m_halfTransparency = false;
    m_orthographicProjection = true;
    m_cameraDistance = 50;
    m_frontClip = 1;
    m_backClip = 100;
    m_FOV = 5;
    m_COIx = m_COIy = m_COIz = 0;
    m_cameraVecX = 0;
    m_cameraVecY = 1;
    m_cameraVecZ = 0;
    m_camera = 0;
    m_trackball = new Trackball();
    m_trackballFlag = false;
    m_panFlag = false;
    m_zoomFlag = false;

    m_cursor3D = 0;
    m_cursorRadius = 0.1f;
    m_cursor3DNudge = 0.1f;
    m_cursorColour = irr::video::SColor(255, 200, 200, 200);

    m_highlightedNode = 0;
    m_aviWriter = 0;
    m_aviQuality = 80;
    m_fps = 25;
}

SimulationWindow::~SimulationWindow()
{
    if (m_aviWriter) delete m_aviWriter;
}

void SimulationWindow::initialiseScene()
{
    sceneManager()->clear();

    irr::s32 state = irr::scene::EDS_OFF;
    if (m_boundingBox) state |= irr::scene::EDS_BBOX;
    if (m_boundingBoxBuffers) state |= irr::scene::EDS_BBOX_BUFFERS;
    if (m_wireFrame) state |= irr::scene::EDS_MESH_WIRE_OVERLAY;
    if (m_normals) state |= irr::scene::EDS_NORMALS;
    if (m_halfTransparency) state |= irr::scene::EDS_HALF_TRANSPARENCY;

    irr::u32 tesselationSphere = 64;
    m_cursor3D = sceneManager()->addSphereSceneNode(m_cursorRadius, tesselationSphere);
    SetMeshColour(m_cursor3D->getMesh(), m_cursorColour);
    m_cursor3D->setDebugDataVisible(state);
    m_cursor3D->setAutomaticCulling(irr::scene::EAC_OFF);

    // create a default camera with an identity view matrix
    m_camera = sceneManager()->addCameraSceneNode(0, irr::core::vector3df(0, 0, 0), irr::core::vector3df(0, 0, 1));
    m_camera->setUpVector(irr::core::vector3df(0, 1, 0));
    sceneManager()->setActiveCamera(m_camera);
    updateCamera();

    // create some light
    sceneManager()->setAmbientLight(irr::video::SColorf(0.3f,0.3f,0.3f));
    irr::scene::ILightSceneNode *light = sceneManager()->addLightSceneNode(0, irr::core::vector3df(0, 0, 0), irr::video::SColorf(.8f, .8f, .8f), 1000.0f);
    light->setLightType(irr::video::ELT_DIRECTIONAL); // directional lights point (0,0,1) by default
    light->setRotation(irr::core::vector3df(-135, 45, 0)); // this should rotate it properly
//    irr::scene::ILightSceneNode *light = sceneManager()->addLightSceneNode(0, irr::core::vector3df(100, -100, 100), irr::video::SColorf(.2f, .2f, .2f), 100.0f);
//    light->setLightType(irr::video::ELT_POINT);
//    light->enableCastShadow(true);

//    sceneManager()->addLightSceneNode(0, irr::core::vector3df(10, 0, 0), irr::video::SColorf(1.0f, 0.0f, 0.0f), 100.0f);
//    sceneManager()->addLightSceneNode(0, irr::core::vector3df(0, 10, 0), irr::video::SColorf(0.0f, 1.0f, 0.0f), 100.0f);
//    sceneManager()->addLightSceneNode(0, irr::core::vector3df(0, 0, 10), irr::video::SColorf(0.0f, 0.0f, 1.0f), 100.0f);

//    // and set the shadow colours
//    sceneManager()->setShadowColor(irr::video::SColor(150,0,0,0));

}

void SimulationWindow::updateCamera()
{
    irr::scene::CCameraSceneNode *camera = dynamic_cast<irr::scene::CCameraSceneNode *>(m_camera);
    if (camera == 0) return;

    // set the projection matrix
    float aspectRatio = (float)width() / (float)height();
    camera->setOrthogonal(m_orthographicProjection);
    camera->setRHCoordinates(true);
    camera->setFOV(irr::core::DEGTORAD * m_FOV);
    camera->setAspectRatio(aspectRatio);
    camera->setNearValue(m_frontClip);
    camera->setFarValue(m_backClip);
    // camera->recalculateProjectionMatrix(); // not needed because all the previous set functions call this internally
    irr::core::vector3df position(m_COIx - m_cameraVecX * m_cameraDistance, m_COIy - m_cameraVecY * m_cameraDistance, m_COIz - m_cameraVecZ * m_cameraDistance);
    irr::core::vector3df target(m_COIx, m_COIy, m_COIz);
    irr::core::vector3df upVector(m_upX, m_upY, m_upZ);
    camera->setPosition(position);
    camera->updateAbsolutePosition();
    camera->setTarget(target);
    camera->setUpVector(upVector);
    camera->updateMatrices();
    m_projectionMatrix = camera->getProjectionMatrix();
    m_viewMatrix = camera->getViewMatrix();
//    if (m_orthographicProjection)
//    {
//        float widthOfViewVolume = m_cameraDistance * sin(float(M_PI/180) * m_FOV);
//        float heightOfViewVolume = widthOfViewVolume / aspectRatio;
//        m_projectionMatrix.buildProjectionMatrixOrthoRH(widthOfViewVolume, heightOfViewVolume, m_frontClip, m_backClip);
//        m_camera->setProjectionMatrix(m_projectionMatrix, true);
//    }
//    else
//    {
//        m_projectionMatrix.buildProjectionMatrixPerspectiveFovRH(float(M_PI/180) * m_FOV, aspectRatio, m_frontClip, m_backClip);
//        m_camera->setProjectionMatrix(m_projectionMatrix, false);
//    }
//    // set the view matrix
//    irr::core::vector3df position(m_COIx - m_cameraVecX * m_cameraDistance, m_COIy - m_cameraVecY * m_cameraDistance, m_COIz - m_cameraVecZ * m_cameraDistance);
//    irr::core::vector3df target(m_COIx, m_COIy, m_COIz);
//    irr::core::vector3df upVector;
//    upVector.set(m_upX, m_upY, m_upZ);
//    m_viewMatrix.buildCameraLookAtMatrixRH(position, target, upVector);
//    m_camera->setViewMatrixAffector(m_viewMatrix);
//    // this may not be necessary but just in case something else has moved the camera
//    position.set(0, 0, 0);
//    target.set(0, 0, 1);
//    upVector.set(0, 1, 0);
//    // viewMatrix.buildCameraLookAtMatrixLH(position, target, upVector); // only used to check my calculations
//    m_camera->setPosition(position);
//    m_camera->setTarget(target);
//    m_camera->setUpVector(upVector);

}

void SimulationWindow::mousePressEvent(QMouseEvent *event)
{
    if (m_simulation == 0) return;

    m_trackballFlag = false;
    if (event->buttons() & Qt::LeftButton)
    {
        if (event->modifiers() == Qt::NoModifier)
        {
            int trackballRadius;
            if (width() < height()) trackballRadius = width() / 2.2;
            else trackballRadius = height() / 2.2;
            m_trackballStartCameraVec.set(m_cameraVecX, m_cameraVecY, m_cameraVecZ);
            m_trackballStartUp.set(m_upX, m_upY, m_upZ);
            m_trackball->StartTrackball(event->pos().x(), event->pos().y(), width() / 2, height() / 2, trackballRadius,
                                        pgd::Vector(m_trackballStartUp.X, m_trackballStartUp.Y, m_trackballStartUp.Z),
                                        pgd::Vector(-m_trackballStartCameraVec.X, -m_trackballStartCameraVec.Y, -m_trackballStartCameraVec.Z));
            m_trackballFlag = true;
            emit EmitStatusString(tr("Rotate"));
            updateCamera();
            renderLater();
        }
        else if (event->modifiers() & Qt::ShiftModifier)
        {
            // detect the collision point of the mouse click
            irr::s32 idBitMask = 1 << 0;
            irr::core::vector3df outCollisionPoint;
            irr::core::triangle3df outTriangle;
            irr::scene::ISceneNode* node = getSceneNodeAndCollisionPointFromRawScreenCoordinates(event->pos().x(), event->pos().y(), idBitMask,
                                                                                                 outCollisionPoint, outTriangle);
           if (m_highlightedNode) m_highlightedNode->setDebugDataVisible(irr::scene::EDS_OFF);
            m_highlightedNode = node;
            if (m_highlightedNode)
            {
                m_cursor3D->setPosition(outCollisionPoint);
                m_highlightedNode->setDebugDataVisible(irr::scene::EDS_BBOX);
            }

            QClipboard *clipboard = QApplication::clipboard();
            clipboard->setText(QString("%1\t%2\t%3").arg(outCollisionPoint.X).arg(outCollisionPoint.Y).arg(outCollisionPoint.Z), QClipboard::Clipboard);
            emit EmitStatusString(QString("3D Cursor %1\t%2\t%3").arg(outCollisionPoint.X).arg(outCollisionPoint.Y).arg(outCollisionPoint.Z));
            updateCamera();
            renderLater();
        }
    }
    else if (event->buttons() & Qt::RightButton)
    {
        if (event->modifiers() == Qt::NoModifier)
        {
            m_panFlag = true;

            m_panStartScreenPoint.set((2.0f * event->pos().x()) / width() - 1.0f, 1.0f - (2.0f * event->pos().y()) / height(), 0);
            m_panStartCOI.set(m_COIx, m_COIy, m_COIz);

            // detect the collision point of the mouse click
            irr::s32 idBitMask = 1 << 0;
            irr::core::vector3df outCollisionPoint;
            irr::core::triangle3df outTriangle;
            irr::scene::ISceneNode* node = getSceneNodeAndCollisionPointFromRawScreenCoordinates(event->pos().x(), event->pos().y(), idBitMask,
                                                                                                 outCollisionPoint, outTriangle);
            if (node) // just use the collision point as the pan reference point
            {
                m_panStartPoint = outCollisionPoint;
                irr::f32 screenStartPoint[4]; // these need to be homogeneous coordinates
                m_projectPanMatrix.transformVect(screenStartPoint, m_panStartPoint);
                m_panStartScreenPoint.Z = screenStartPoint[2]/screenStartPoint[3];
            }
            else // this is harder since we don't know the screen Z were are interested in.
            {
                // generate a screen Z from projecting the COI into screen coordinates (-1 to 1 box)
                irr::f32 screenStartPoint[4]; // these need to be homogeneous coordinates
                m_projectPanMatrix.transformVect(screenStartPoint, m_panStartCOI);
                m_panStartScreenPoint.Z = screenStartPoint[2]/screenStartPoint[3];
                // now unproject this point to get the pan start point
                irr::f32 startPoint[4]; // these need to be homogeneous coordinates
                m_unprojectPanMatrix.transformVect(startPoint, m_panStartScreenPoint); // this is now in world coordinates
                m_panStartPoint.set(startPoint[0]/startPoint[3], startPoint[1]/startPoint[3], startPoint[2]/startPoint[3]);
            }
            //qDebug("m_panStartPoint=%f,%f,%f", m_panStartPoint.X, m_panStartPoint.Y, m_panStartPoint.Z);

            emit EmitStatusString(tr("Pan"));
            updateCamera();
            renderLater();
        }
        else if (event->modifiers() & Qt::AltModifier)
        {
            // detect the collision point of the mouse click
            irr::s32 idBitMask = 1 << 0;
            irr::core::vector3df outCollisionPoint;
            irr::core::triangle3df outTriangle;
            irr::scene::ISceneNode* node = getSceneNodeAndCollisionPointFromRawScreenCoordinates(event->pos().x(), event->pos().y(), idBitMask,
                                                                                                 outCollisionPoint, outTriangle);
            if (node)
            {
                m_COIx = outCollisionPoint.X;
                m_COIy = outCollisionPoint.Y;
                m_COIz = outCollisionPoint.Z;
            }
            QClipboard *clipboard = QApplication::clipboard();
            clipboard->setText(QString("%1\t%2\t%3").arg(outCollisionPoint.X).arg(outCollisionPoint.Y).arg(outCollisionPoint.Z), QClipboard::Clipboard);
            emit EmitStatusString(QString("Centre of Interest %1\t%2\t%3").arg(outCollisionPoint.X).arg(outCollisionPoint.Y).arg(outCollisionPoint.Z));
            emit EmitCOI(outCollisionPoint.X, outCollisionPoint.Y, outCollisionPoint.Z);
            updateCamera();
            renderLater();
        }
    }
    else if (event->buttons() & Qt::MidButton)
    {
        if (event->modifiers() == Qt::NoModifier)
        {
            m_zoomFlag = true;
            // centred -1 to -1 normalised values
            double x = (double)(2 * event->pos().x()) / (double)width() - 1.0;
            double y = (double)(2 * (height() - event->pos().y())) / (double)height() - 1.0;
            m_zoomDistance = sqrt(x * x + y * y);
            if (m_zoomDistance < 0.05) m_zoomDistance = 0.05;
            m_zoomStartFOV = m_FOV;

            emit EmitStatusString(tr("Zoom"));
            updateCamera();
            renderLater();
        }
    }

}

void SimulationWindow::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton)
    {
        if (m_trackballFlag)
        {
            pgd::Quaternion pgdRotation;
            m_trackball->RollTrackballToClick(event->pos().x(), event->pos().y(), &pgdRotation);
            irr::core::quaternion rotation(pgdRotation.v.x, pgdRotation.v.y, pgdRotation.v.z, pgdRotation.n);
            rotation.makeInverse();
            irr::core::vector3df newCameraVec = rotation * m_trackballStartCameraVec;
            m_cameraVecX = newCameraVec.X;
            m_cameraVecY = newCameraVec.Y;
            m_cameraVecZ = newCameraVec.Z;
            irr::core::vector3df newUp = rotation *  m_trackballStartUp;
            m_upX = newUp.X;
            m_upY = newUp.Y;
            m_upZ = newUp.Z;
            updateCamera();
            renderLater();

            emit EmitStatusString(QString("Camera %1 %2 %3 Up %4 %5 %6").arg(m_cameraVecX).arg(m_cameraVecY).arg(m_cameraVecZ).arg(m_upX).arg(m_upY).arg(m_upZ));
        }
    }
    else if (event->buttons() & Qt::RightButton)
    {
        if (m_panFlag)
        {
            irr::core::vector3df screenPoint((2.0f * event->pos().x()) / width() - 1.0f, 1.0f - (2.0f * event->pos().y()) / height(), m_panStartScreenPoint.Z);
            irr::f32 panCurrentPoint[4];
            m_unprojectPanMatrix.transformVect(panCurrentPoint, screenPoint);
            m_COIx = m_panStartCOI.X - (panCurrentPoint[0]/panCurrentPoint[3] - m_panStartPoint.X);
            m_COIy = m_panStartCOI.Y - (panCurrentPoint[1]/panCurrentPoint[3] - m_panStartPoint.Y);
            m_COIz = m_panStartCOI.Z - (panCurrentPoint[2]/panCurrentPoint[3] - m_panStartPoint.Z);
            // qDebug("panCurrentPoint=%f,%f,%f,%f", panCurrentPoint[0], panCurrentPoint[1], panCurrentPoint[2], panCurrentPoint[3]);

            emit EmitStatusString(QString("COI %1 %2 %3").arg(m_COIx).arg(m_COIy).arg(m_COIz));
            emit EmitCOI(m_COIx, m_COIy, m_COIz);
            updateCamera();
            renderLater();
        }
    }
    else if (event->buttons() & Qt::MidButton)
    {
        if (m_zoomFlag)
        {
            // centred -1 to -1 normalised values
            double x = (double)(2 * event->pos().x()) / (double)width() - 1.0;
            double y = (double)(2 * (height() - event->pos().y())) / (double)height() - 1.0;
            double zoomDistance = sqrt(x * x + y * y);
            m_FOV = m_zoomStartFOV * m_zoomDistance / zoomDistance;
            if (m_FOV > 170) m_FOV = 170;
            else if (m_FOV < 0.001) m_FOV = 0.001f;
            updateCamera();
            renderLater();

            emit EmitStatusString(QString("FOV %1").arg(m_FOV));
            emit EmitFoV(m_FOV);
        }
    }
}

void SimulationWindow::mouseReleaseEvent(QMouseEvent * /* event */)
{
    m_trackballFlag = false;
    m_panFlag = false;
    m_zoomFlag = false;
    updateCamera();
    renderLater();
}

void SimulationWindow::wheelEvent(QWheelEvent * event)
{
    // assume each ratchet of the wheel gives a score of 120 (8 * 15 degrees)
    double sensitivity = 2400;
    double scale = 1.0 + double(event->delta()) / sensitivity;
    m_FOV *= scale;
    if (m_FOV > 170) m_FOV = 170;
    else if (m_FOV < 0.001) m_FOV = 0.001f;
    updateCamera();
    renderLater();

    emit EmitStatusString(QString("FOV %1").arg(m_FOV));
    emit EmitFoV(m_FOV);
}

// handle key presses
void SimulationWindow::keyPressEvent(QKeyEvent *event)
{
    irr::core::vector3df newPosition = m_cursor3D->getPosition();
    switch( event->key() )
    {

        // X, Y and Z move the cursor
    case Qt::Key_X:
        if (event->modifiers() == Qt::NoModifier)
            newPosition.X += m_cursor3DNudge;
        else
            newPosition.X -= m_cursor3DNudge;
        break;

    case Qt::Key_Y:
        if (event->modifiers() == Qt::NoModifier)
            newPosition.Y += m_cursor3DNudge;
        else
            newPosition.Y -= m_cursor3DNudge;
        break;

    case Qt::Key_Z:
        if (event->modifiers() == Qt::NoModifier)
            newPosition.Z += m_cursor3DNudge;
        else
            newPosition.Z -= m_cursor3DNudge;
        break;

        // S snaps the cursor to the nearest whole number multiple of the nudge value
    case Qt::Key_S:
        newPosition.X = round(newPosition.X / m_cursor3DNudge) * m_cursor3DNudge;
        newPosition.Y = round(newPosition.Y / m_cursor3DNudge) * m_cursor3DNudge;
        newPosition.Z = round(newPosition.Z / m_cursor3DNudge) * m_cursor3DNudge;
        break;
    }

    if (newPosition != m_cursor3D->getPosition())
    {
        m_cursor3D->setPosition(newPosition);
        QClipboard *clipboard = QApplication::clipboard();
        clipboard->setText(QString("%1\t%2\t%3").arg(newPosition.X).arg(newPosition.Y).arg(newPosition.Z), QClipboard::Clipboard);
        emit EmitStatusString(QString("3D Cursor %1\t%2\t%3").arg(newPosition.X).arg(newPosition.Y).arg(newPosition.Z));
        updateCamera();
        renderLater();
    }
}

int SimulationWindow::aviQuality() const
{
    return m_aviQuality;
}

void SimulationWindow::setAviQuality(int aviQuality)
{
    m_aviQuality = aviQuality;
}

AVIWriter *SimulationWindow::aviWriter() const
{
    return m_aviWriter;
}

void SimulationWindow::setAviWriter(AVIWriter *aviWriter)
{
    m_aviWriter = aviWriter;
}

irr::video::SColor SimulationWindow::cursorColour() const
{
    return m_cursorColour;
}

void SimulationWindow::setCursorColour(const irr::video::SColor &cursorColour)
{
    m_cursorColour = cursorColour;
}

// collide with an object based on the Qt screen coordinates
irr::scene::ISceneNode* SimulationWindow::getSceneNodeAndCollisionPointFromRawScreenCoordinates(int screenX, int screenY, irr::s32 idBitMask,
                                                                                                irr::core::vector3df &outCollisionPoint, irr::core::triangle3df &outTriangle)
{
    //qDebug("width=%d height=%d x=%d y=%d", width(), height(), screenX, screenY);
    irr::core::vector3df startPointScreen((2.0f * screenX) / width() - 1.0f, 1.0f - (2.0f * screenY) / height(), -1);
    irr::core::vector3df endPointScreen((2.0f * screenX) / width() - 1.0f, 1.0f - (2.0f * screenY) / height(), 1);
    //qDebug("startPointScreen=%f,%f,%f endPointScreen=%f,%f,%f", startPointScreen.X, startPointScreen.Y, startPointScreen.Z, endPointScreen.X, endPointScreen.Y, endPointScreen.Z);
    irr::f32 startPoint[4], endPoint[4]; // these need to be homogeneous coordinates
    m_projectPanMatrix = m_projectionMatrix * m_viewMatrix; // model would be identity so mvpMatrix isn't needed
    m_projectPanMatrix.getInverse(m_unprojectPanMatrix); // we need the unproject matrix
    m_unprojectPanMatrix.transformVect(startPoint, startPointScreen); // this is now in world coordinates
    m_unprojectPanMatrix.transformVect(endPoint, endPointScreen);
    irr::scene::ISceneCollisionManager* collMan = sceneManager()->getSceneCollisionManager();
    irr::core::line3d<irr::f32> ray(startPoint[0]/startPoint[3], startPoint[1]/startPoint[3], startPoint[2]/startPoint[3],
            endPoint[0]/endPoint[3], endPoint[1]/endPoint[3], endPoint[2]/endPoint[3]);
    //qDebug("ray=%f,%f,%f to %f,%f,%f", ray.start.X, ray.start.Y, ray.start.Z, ray.end.X, ray.end.Y, ray.end.Z);
    irr::scene::ISceneNode* node = collMan->getSceneNodeAndCollisionPointFromRay(ray, outCollisionPoint, outTriangle, idBitMask);
    return node;
}

// write the current frame out to a file
int SimulationWindow::WriteStillFrame(const QString &filename)
{
    // get image from the last rendered frame
    irr::video::IImage* const image = videoDriver()->createScreenShot();
    if (image == 0)
    {
        qDebug("Error: SimulationWindow::WriteStillFrame(%s) cannot createScreenShot\n", qPrintable(filename));
        return __LINE__;
    }
    // write screenshot to file
    QString filenameWithExtension = filename + ".png";
#ifdef _IRR_WCHAR_FILESYSTEM
    if (!videoDriver()->writeImageToFile(image, filenameWithExtension.toStdWString().c_str()))
#else
    if (!videoDriver()->writeImageToFile(image, filenameWithExtension.toUtf8().data()))
#endif
    {
        qDebug("Error: SimulationWindow::WriteStillFrame(%s) cannot writeImageToFile\n", qPrintable(filename));
        return __LINE__;
    }
    image->drop();

    return 0;
}

// write the current frame out to a file
int SimulationWindow::WriteMovieFrame()
{
    Q_ASSERT(m_aviWriter);
    // get image from the last rendered frame
    irr::video::IImage* const image = videoDriver()->createScreenShot();
    if (image) //should always be true, but you never know. ;)
    {
        irr::core::dimension2d<irr::u32>size =  image->getDimension();
        unsigned char *rgb = new unsigned char[size.Width * size.Height * 3];
        unsigned char *ptr = rgb;
        irr::video::SColor c;
        for (unsigned int iy = 0; iy < size.Height; iy++)
        {
            for (unsigned int ix = 0; ix < size.Width; ix++)
            {
                c = image->getPixel(ix, iy);
                *ptr++ = c.getRed();
                *ptr++ = c.getGreen();
                *ptr++ = c.getBlue();
            }
        }
        image->drop();
        m_aviWriter->WriteAVI(size.Width, size.Height, rgb, m_aviQuality);
        delete [] rgb;
    }
    return 0;
}

int SimulationWindow::StartAVISave(const QString &fileName)
{
    if (m_aviWriter) delete m_aviWriter;
    m_aviWriter = new AVIWriter();
    if (m_aviQuality == 0) return __LINE__; // should always be true
    irr::video::IImage* const image = videoDriver()->createScreenShot();
    if (image == 0) return __LINE__; //should always be true, but you never know. ;)
    irr::core::dimension2d<irr::u32>size =  image->getDimension();
    unsigned char *rgb = new unsigned char[size.Width * size.Height * 3];
    unsigned char *ptr = rgb;
    irr::video::SColor c;
    for (unsigned int iy = 0; iy < size.Height; iy++)
    {
        for (unsigned int ix = 0; ix < size.Width; ix++)
        {
            c = image->getPixel(ix, iy);
            *ptr++ = c.getRed();
            *ptr++ = c.getGreen();
            *ptr++ = c.getBlue();
        }
    }
    image->drop();
    m_aviWriter->InitialiseFile(fileName, size.Width, size.Height, m_fps);

    m_aviWriter->WriteAVI(size.Width, size.Height, rgb, m_aviQuality);
    delete [] rgb;
    return 0;
}

int SimulationWindow::StopAVISave()
{
    if (m_aviWriter == 0) return __LINE__;
    delete m_aviWriter;
    m_aviWriter = 0;
    return 0;
}

// write the scene as a series of OBJ files in a folder
int SimulationWindow::WriteCADFrame(const QString &pathname)
{
    QString workingFolder = QDir::currentPath();
    if (QDir(pathname).exists() == false)
    {
        if (QDir().mkdir(pathname) == false)
        {
            QMessageBox::warning(0, "Snapshot Error", QString("Could not create folder '%1' for OBJ files\n").arg(pathname), "Click button to return to simulation");
            return __LINE__;
        }
    }
    QDir::setCurrent(pathname);

    irr::scene::ISceneNode *rootSceneNode = sceneManager()->getRootSceneNode();
    irr::scene::ISceneNodeList allSceneNodes;
    GetAllChildren(rootSceneNode, &allSceneNodes);

    irr::scene::IMeshWriter *meshWriter = sceneManager()->createMeshWriter(irr::scene::EMWT_OBJ);
    irr::io::IFileSystem *fs = sceneManager()->getFileSystem();
    irr::scene::IMeshManipulator *mm = sceneManager()->getMeshManipulator(); // careful because only some IMeshManipulator functions work on IDynamicMesh and EIT_32BIT index types

    irr::core::array<irr::scene::IMesh *> meshList;
    irr::core::array<const irr::core::matrix4 *> transformationList;
    irr::core::array<const irr:: c8 *> nameList;
    for (irr::scene::ISceneNodeList::ConstIterator it = allSceneNodes.begin(); it != allSceneNodes.end(); it++)
    {
        irr::scene::IMeshSceneNode *meshSceneNode = dynamic_cast<irr::scene::IMeshSceneNode *>(*it);
        if (meshSceneNode)
        {
            meshList.push_back(meshSceneNode->getMesh());
            meshSceneNode->updateAbsolutePosition();
            transformationList.push_back(&meshSceneNode->getAbsoluteTransformation());
            continue;
        }
        irr::scene::IAnimatedMeshSceneNode *animatedMeshSceneNode = dynamic_cast<irr::scene::IAnimatedMeshSceneNode *>(*it);
        if (animatedMeshSceneNode)
        {
            meshList.push_back(animatedMeshSceneNode->getMesh()->getMesh(0));
            animatedMeshSceneNode->updateAbsolutePosition();
            transformationList.push_back(&animatedMeshSceneNode->getAbsoluteTransformation());
            continue;
        }
    }

    for (irr::u32 meshCount = 0; meshCount < meshList.size(); meshCount++)
    {

        QString meshFileName = QString("Mesh%1.obj").arg(meshCount, 5, 10, QChar('0'));
        irr::io::IWriteFile *file = fs->createAndWriteFile(meshFileName.toUtf8().constData());
        irr::core::matrix4 transformation = *transformationList[meshCount];
        mm->transform(meshList[meshCount], transformation); // transform the mesh in place to avoid duplicating a big mesh
        meshWriter->writeMesh(file, meshList[meshCount]);
        transformation.makeInverse();
        mm->transform(meshList[meshCount], transformation); // and reverse the transformation
        file->drop();

        for (irr::u32 meshBufferIndex = 0; meshBufferIndex <  meshList[meshCount]->getMeshBufferCount(); meshBufferIndex++)
        {
            const irr::video::SMaterial &material =  meshList[meshCount]->getMeshBuffer(meshBufferIndex)->getMaterial();
            irr::video::ITexture *texture = material.getTexture(0);
            if (texture)
            {
                irr::io::path materialPath = texture->getName();
                qDebug() << "materialPath " << materialPath.c_str() << "\n";
                irr::io::path textureBasename = fs->getFileBasename(materialPath);
                irr::video::ECOLOR_FORMAT format = texture->getColorFormat();
                if (format == irr::video::ECF_A8R8G8B8)
                {
                    const irr::core::dimension2du &textureSize = texture->getSize();
                    QImage qImage(textureSize.Width, textureSize.Height, QImage::Format_ARGB32);
                    irr::u8 *texturePtr = static_cast<irr::u8 *>(texture->lock(irr::video::ETLM_READ_ONLY, 0));
                    memcpy(qImage.bits(), texturePtr, qImage.width() * qImage.height() * 4);
                    texture->unlock();
                    QImage flippedImage = qImage.mirrored(false, true);
#ifdef _IRR_WCHAR_FILESYSTEM
                    flippedImage.save(QString::fromWCharArray(textureBasename.c_str()));
#else
                    flippedImage.save(QString(textureBasename.c_str()));
#endif
                }
                else
                {
                    QMessageBox::warning(0, "Snapshot Error", QString("Could note generate texture for format \1\n").arg(format), "Click button to continue");
                }
            }
        }
    }

    meshWriter->drop();
    QDir::setCurrent(workingFolder);
    return 0;
}

void SimulationWindow::GetAllChildren(irr::scene::ISceneNode *sceneNode, irr::core::list<irr::scene::ISceneNode *> *sceneNodeList)
{
    const irr::scene::ISceneNodeList &children = sceneNode->getChildren();
    if (children.size() == 0) return;
    for (irr::scene::ISceneNodeList::ConstIterator it = children.begin(); it != children.end(); it++)
    {
        GetAllChildren(*it, sceneNodeList);
        sceneNodeList->push_back(*it);
    }
}

void SimulationWindow::SetCameraVec(double x, double y, double z)
{
    m_cameraVecX = x;
    m_cameraVecY = y;
    m_cameraVecZ = z;
    if (z > 0.999 || z < -0.999)
    {
        m_upX = 0;
        m_upY = 1;
        m_upZ = 0;
    }
    else
    {
        m_upX = 0;
        m_upY = 0;
        m_upZ = 1;
    }
    updateCamera();
    renderLater();
}

// set up all the things needed to get the colour right
void SimulationWindow::SetMeshColour(irr::scene::IMesh *mesh, const irr::video::SColor &colour)
{
    // need to set both the material and the vertex colours to make this work properly
    // the vertex colours are what are mostly used but the material colour is used for OBJ export (for example)

    // calculate some sensible proportions
    irr::u32 r = colour.getRed();
    irr::u32 g = colour.getGreen();
    irr::u32 b = colour.getBlue();
    irr::u32 a = colour.getAlpha();
    float ambientProportion = 0.2f;
    float diffuseProportion = 0.8f;
    float specularProportion = 0.3f;
    float emissiveProportion = 0.0f;
    float shininess = 20.0f;

    irr::video::SMaterial material; // this class is deep copied and does not use reference counting

    // this lets me see what all the options are (defaults are in the comments)
    material.MaterialType = irr::video::EMT_SOLID; // MaterialType(EMT_SOLID)

    material.AmbientColor.set(a * ambientProportion, r * ambientProportion, g * ambientProportion, b * ambientProportion); // AmbientColor(255,255,255,255)
    material.DiffuseColor.set(a * diffuseProportion, r * diffuseProportion, g * diffuseProportion, b * diffuseProportion); // DiffuseColor(255,255,255,255)
    material.EmissiveColor.set(a * emissiveProportion, r * emissiveProportion, g * emissiveProportion, b * emissiveProportion); // EmissiveColor(0,0,0,0)
    material.SpecularColor.set(a * specularProportion, r * specularProportion, g * specularProportion, b * specularProportion); // SpecularColor(255,255,255,255)
    material.Shininess = shininess; // Shininess(0.0f)
    material.MaterialTypeParam = 0; // MaterialTypeParam(0.0f)
    material.MaterialTypeParam2 = 0; // MaterialTypeParam2(0.0f)
    material.Thickness = 1; // Thickness(1.0f)
    // for (u32 i=0; i<MATERIAL_MAX_TEXTURES; ++i) material.TextureLayer[i] left as default

    material.Wireframe = false; // Wireframe(false)
    material.PointCloud = false; // PointCloud(false)
    material.GouraudShading = false; // GouraudShading(true)
    material.Lighting = true; // Lighting(true)
    material.ZWriteEnable = true; // ZWriteEnable(true)
    material.BackfaceCulling = false; // BackfaceCulling(true)
    material.FrontfaceCulling = true; // FrontfaceCulling(false)
    material.FogEnable = false; // FogEnable(false)
    material.NormalizeNormals = true; // NormalizeNormals(false)
    material.ZBuffer = irr::video::ECFN_LESSEQUAL; // ZBuffer(ECFN_LESSEQUAL)
    material.AntiAliasing = irr::video::EAAM_SIMPLE; // AntiAliasing(EAAM_SIMPLE)
    material.ColorMask = irr::video::ECP_ALL; // ColorMask(ECP_ALL)
    material.ColorMaterial = irr::video::ECM_DIFFUSE; // ColorMaterial(ECM_DIFFUSE)
    material.BlendOperation = irr::video::EBO_NONE; // BlendOperation(EBO_NONE)
    material.BlendFactor = 0; // BlendFactor(0.0f)
    material.PolygonOffsetFactor = 0; // PolygonOffsetFactor(0)
    material.PolygonOffsetDirection = irr::video::EPO_FRONT; // PolygonOffsetDirection(EPO_FRONT)
    material.UseMipMaps = true; // UseMipMaps(true)
    material.ZWriteFineControl = irr::video::EZI_ONLY_NON_TRANSPARENT; // ZWriteFineControl(EZI_ONLY_NON_TRANSPARENT)

    irr::scene::IMeshManipulator *mm = sceneManager()->getMeshManipulator();
    mm->setVertexColors(mesh, irr::video::SColor(a, r, g, b));

    for (irr::u32 i = 0; i < mesh->getMeshBufferCount(); i++)
    {
        irr::video::SMaterial &materialRef = mesh->getMeshBuffer(i)->getMaterial();
        materialRef = material;
    }

//    irr::scene::SMesh *sMesh = dynamic_cast<irr::scene::SMesh *>(mesh);
//    if (sMesh)
//    {
//        for (irr::u32 i = 0; i < sMesh->getMeshBufferCount(); i++)
//        {
//            irr::video::SMaterial &materialRef = sMesh->getMeshBuffer(i)->getMaterial();
//            materialRef = material;
//        }
//        return;
//    }

//    irr::scene::SAnimatedMesh *animatedMesh = dynamic_cast<irr::scene::SAnimatedMesh *>(mesh);
//    if (animatedMesh)
//    {

//        for (irr::u32 j = 0; j < animatedMesh->getFrameCount(); j++)
//        {
//            for (irr::u32 i = 0; i < animatedMesh->getMesh(j)->getMeshBufferCount(); i++)
//            {
//                irr::video::SMaterial &materialRef = animatedMesh->getMesh(j)->getMeshBuffer(i)->getMaterial();
//                materialRef = material;
//            }
//        }
//        return;
//    }

//    qDebug() << "Unrecognised mesh in SetMeshColour\n";

}

irr::scene::ISceneNode *SimulationWindow::highlightedNode() const
{
    return m_highlightedNode;
}

void SimulationWindow::setHighlightedNode(irr::scene::ISceneNode *highlightedNode)
{
    m_highlightedNode = highlightedNode;
}

bool SimulationWindow::halfTransparency() const
{
    return m_halfTransparency;
}

void SimulationWindow::setHalfTransparency(bool halfTransparency)
{
    m_halfTransparency = halfTransparency;
}

bool SimulationWindow::normals() const
{
    return m_normals;
}

void SimulationWindow::setNormals(bool normals)
{
    m_normals = normals;
}

float SimulationWindow::cursor3DNudge() const
{
    return m_cursor3DNudge;
}

void SimulationWindow::setCursor3DNudge(float cursor3DNudge)
{
    m_cursor3DNudge = cursor3DNudge;
}

float SimulationWindow::cursorRadius() const
{
    return m_cursorRadius;
}

void SimulationWindow::setCursorRadius(float cursorRadius)
{
    m_cursorRadius = cursorRadius;
}

float SimulationWindow::upZ() const
{
    return m_upZ;
}

void SimulationWindow::setUpZ(float upZ)
{
    m_upZ = upZ;
}

float SimulationWindow::upY() const
{
    return m_upY;
}

void SimulationWindow::setUpY(float upY)
{
    m_upY = upY;
}

float SimulationWindow::upX() const
{
    return m_upX;
}

void SimulationWindow::setUpX(float upX)
{
    m_upX = upX;
}

Simulation *SimulationWindow::simulation() const
{
    return m_simulation;
}

void SimulationWindow::setSimulation(Simulation *simulation)
{
    m_simulation = simulation;
}

bool SimulationWindow::wireFrame() const
{
    return m_wireFrame;
}

void SimulationWindow::setWireFrame(bool wireFrame)
{
    m_wireFrame = wireFrame;
}

bool SimulationWindow::boundingBox() const
{
    return m_boundingBox;
}

void SimulationWindow::setBoundingBox(bool boundingBox)
{
    m_boundingBox = boundingBox;
}

bool SimulationWindow::orthographicProjection() const
{
    return m_orthographicProjection;
}

void SimulationWindow::setOrthographicProjection(bool orthographicProjection)
{
    m_orthographicProjection = orthographicProjection;
}

float SimulationWindow::cameraDistance() const
{
    return m_cameraDistance;
}

void SimulationWindow::setCameraDistance(float cameraDistance)
{
    m_cameraDistance = cameraDistance;
}

float SimulationWindow::FOV() const
{
    return m_FOV;
}

void SimulationWindow::setFOV(float FOV)
{
    m_FOV = FOV;
}

float SimulationWindow::cameraVecX() const
{
    return m_cameraVecX;
}

void SimulationWindow::setCameraVecX(float cameraVecX)
{
    m_cameraVecX = cameraVecX;
}

float SimulationWindow::cameraVecY() const
{
    return m_cameraVecY;
}

void SimulationWindow::setCameraVecY(float cameraVecY)
{
    m_cameraVecY = cameraVecY;
}

float SimulationWindow::cameraVecZ() const
{
    return m_cameraVecZ;
}

void SimulationWindow::setCameraVecZ(float cameraVecZ)
{
    m_cameraVecZ = cameraVecZ;
}

float SimulationWindow::COIx() const
{
    return m_COIx;
}

void SimulationWindow::setCOIx(float COIx)
{
    m_COIx = COIx;
}

float SimulationWindow::COIy() const
{
    return m_COIy;
}

void SimulationWindow::setCOIy(float COIy)
{
    m_COIy = COIy;
}

float SimulationWindow::COIz() const
{
    return m_COIz;
}

void SimulationWindow::setCOIz(float COIz)
{
    m_COIz = COIz;
}

float SimulationWindow::frontClip() const
{
    return m_frontClip;
}

void SimulationWindow::setFrontClip(float frontClip)
{
    m_frontClip = frontClip;
}

float SimulationWindow::backClip() const
{
    return m_backClip;
}

void SimulationWindow::setBackClip(float backClip)
{
    m_backClip = backClip;
}

