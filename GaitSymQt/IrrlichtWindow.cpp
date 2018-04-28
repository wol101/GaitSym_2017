#include <QtCore/QCoreApplication>
#include <QResizeEvent>

//#include <gl.h>

#include "IrrlichtWindow.h"

IrrlichtWindow::IrrlichtWindow(QWindow *parent)
    : QWindow(parent)
{
    m_update_pending = false;

    m_irrlichtDevice = 0;
    m_videoDriver = 0;
    m_sceneManager = 0;

    m_backgroundColour = irr::video::SColor(255,100,101,140);
//    m_aspectRatio = 1;
//    m_resizeTimerId = 0;
}

IrrlichtWindow::~IrrlichtWindow()
{
    if (m_irrlichtDevice) m_irrlichtDevice->drop();
}

void IrrlichtWindow::render(QPainter *painter)
{
    Q_UNUSED(painter);
}

void IrrlichtWindow::initializeIrrlicht()
{
    irr::SIrrlichtCreationParameters createParams;

    // set the defaults for the device
    createParams.DeviceType = irr::EIDT_BEST;
#ifdef USE_SOFTWARE_RENDERER
    createParams.DriverType = irr::video::EDT_BURNINGSVIDEO; //irr::video::EDT_OPENGL;
#else
    createParams.DriverType = irr::video::EDT_OPENGL;
#endif
    createParams.WindowId = (void *) this->winId();
    createParams.WindowSize = irr::core::dimension2d<irr::u32>(this->width(), this->height());
    createParams.Bits = 32;
    createParams.ZBufferBits = 24;
    createParams.Fullscreen = false;
    createParams.Stencilbuffer = true;
    createParams.Vsync = true;
    createParams.AntiAlias = 8;
    createParams.HandleSRGB = false;
    createParams.WithAlphaChannel = true;
    createParams.Doublebuffer = true;
    createParams.IgnoreInput = false;
    createParams.Stereobuffer = false;
    createParams.HighPrecisionFPU = true;
    createParams.EventReceiver = 0;
    createParams.DisplayAdapter = 0;
    createParams.DriverMultithreaded = true;
    createParams.UsePerformanceTimer = true;

    if (m_irrlichtDevice) m_irrlichtDevice->drop();
    m_irrlichtDevice = irr::createDeviceEx(createParams);
    m_videoDriver = m_irrlichtDevice->getVideoDriver();
    m_sceneManager = m_irrlichtDevice->getSceneManager();

    // set some useful defaults for the driver
    m_videoDriver->setTextureCreationFlag(irr::video::ETCF_ALWAYS_32_BIT, true);
    m_videoDriver->setTextureCreationFlag(irr::video::ETCF_ALLOW_NON_POWER_2, true);
    m_videoDriver->setTextureCreationFlag(irr::video::ETCF_ALLOW_MEMORY_COPY, true);
    m_videoDriver->setTextureCreationFlag(irr::video::ETCF_CREATE_MIP_MAPS, true);
}

void IrrlichtWindow::render()
{
    if (m_videoDriver && m_sceneManager)
    {
        m_irrlichtDevice->getTimer()->tick(); // need to do this by hand because we are not using the irr::IrrlichtDevice::run() method

        m_videoDriver->beginScene(static_cast<irr::u16>(irr::video::ECBF_COLOR | irr::video::ECBF_DEPTH), m_backgroundColour);
        m_sceneManager->drawAll();
        m_videoDriver->endScene();
    }
}

void IrrlichtWindow::renderLater()
{
    if (!m_update_pending) {
        m_update_pending = true;
        QCoreApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
    }
}

bool IrrlichtWindow::event(QEvent *event)
{
    switch (event->type()) {
    case QEvent::UpdateRequest:
        m_update_pending = false;
        renderNow();
        return true;
    default:
        return QWindow::event(event);
    }
}

void IrrlichtWindow::exposeEvent(QExposeEvent *event)
{
    Q_UNUSED(event);

    if (isExposed())
        renderNow();
}

void IrrlichtWindow::renderNow()
{
    if (!isExposed())
        return;

    render();
}

void IrrlichtWindow::resizeEvent(QResizeEvent *event)
{
    if (m_irrlichtDevice)
    {
//        GLint viewport[4];
//        glGetIntegerv(GL_VIEWPORT, viewport);
//        qDebug("x=%d y=%d w=%d h=%d", viewport[0], viewport[1], viewport[2], viewport[3]);
//        qDebug("oldw=%d oldh=%d neww=%d newhh=%d", event->oldSize().width(), event->oldSize().height(), event->size().width(), event->size().height());

        irr::core::dimension2d<irr::u32> size;
        size.Width = event->size().width();
        size.Height = event->size().height();
        m_irrlichtDevice->getVideoDriver()->OnResize(size);
    }
}


//void IrrlichtWindow::resizeEvent(QResizeEvent *event)
//{
//    m_aspectRatio = (float)event->size().width() / (float)event->size().height();
//    if (m_driver && m_smgr)
//    {
//        // this is done so we only update after the resize has finished
//        if (m_resizeTimerId)
//        {
//            killTimer(m_resizeTimerId);
//            m_resizeTimerId = 0;
//        }
//        m_resizeTimerId = startTimer(100); // this is the delay between ends of resize and the repaint actions
//        qDebug("Resizing %d %d", event->size().width(), event->size().height());
//    }
//    QWindow::resizeEvent(event);
//}

//void IrrlichtWindow::timerEvent(QTimerEvent *event)
//{
//    if (event->timerId() == m_resizeTimerId) // we have reached the end of a resize activity
//    {
//        qDebug("End of resize m_resizeTimerId=%d", m_resizeTimerId);
//        killTimer(event->timerId());
//        m_resizeTimerId = 0;

//        irr::core::list<irr::scene::ISceneNode *> nodeList(m_smgr->getRootSceneNode()->getChildren()); // this copies all the nodes into a new list
//        irr::core::list<irr::scene::ISceneNode*>::ConstIterator it;
//        for (it = nodeList.begin(); it != nodeList.end(); it++) (*it)->grab(); // this increases the reference counts
//        initializeIrrlicht();
//        for (it = nodeList.begin(); it != nodeList.end(); it++) m_smgr->getRootSceneNode()->addChild(*it);
//        renderLater();
//    }
//}

//float IrrlichtWindow::aspectRatio() const
//{
//    return m_aspectRatio;
//}

irr::video::SColor IrrlichtWindow::backgroundColour() const
{
    return m_backgroundColour;
}

void IrrlichtWindow::setBackgroundColour(const irr::video::SColor &BackgroundColour)
{
    m_backgroundColour = BackgroundColour;
}

irr::scene::ISceneManager *IrrlichtWindow::sceneManager() const
{
    return m_sceneManager;
}

irr::video::IVideoDriver *IrrlichtWindow::videoDriver() const
{
    return m_videoDriver;
}

irr::IrrlichtDevice *IrrlichtWindow::irrlichtDevice() const
{
    return m_irrlichtDevice;
}

