#include "AVIWriter.h"

#include "gwavi.h"

#include <QDebug>
#include <QImage>
#include <QByteArray>
#include <QBuffer>

AVIWriter::AVIWriter()
{
    m_gwavi = 0;
    m_width = 0;
    m_height = 0;
    m_fps = 0;
}

AVIWriter::~AVIWriter()
{
    if (m_gwavi) CloseFile();
}

int AVIWriter::InitialiseFile(const QString &aviFilename, unsigned int width, unsigned int height, unsigned int fps)
{
    m_width = width;
    m_height = height;
    m_fps = fps;
    m_aviFilename = aviFilename;

    const char *fourcc = "MJPG";		  /* set fourcc used */

#ifdef Q_OS_WIN
    m_gwavi = gwavi_open(aviFilename.toStdWString().c_str(), m_width, m_height, fourcc, m_fps, 0);
#else
    m_gwavi = gwavi_open(aviFilename.toUtf8().constData(), m_width, m_height, fourcc, m_fps, 0);
#endif

    if (!m_gwavi)
    {
        qDebug("Error: call to gwavi_open(%s) failed!\n", qPrintable(m_aviFilename));
        return __LINE__;
    }
    return 0;
}

int AVIWriter::WriteAVI(unsigned int width, unsigned int height, const unsigned char *rgb, int quality)
{
    if (width != m_width || height != m_height)
    {
        qDebug("Error: width or height has changed since starting video %s\n", qPrintable(m_aviFilename));
        return __LINE__;
    }
    QImage image(rgb, m_width, m_height, m_width * 3, QImage::Format_RGB888);
    QByteArray ba;
    QBuffer buffer(&ba);
    buffer.open(QIODevice::WriteOnly);
    image.save(&buffer, "JPG", quality); // this saves the JPG file data to a QByteArray

    if (gwavi_add_frame(m_gwavi, reinterpret_cast<const unsigned char *>(buffer.data().data()), buffer.size()) == -1)
    {
        qDebug("Error: cannot add frame to video %s\n", qPrintable(m_aviFilename));
        return __LINE__;
    }
    return 0;
}

int AVIWriter::CloseFile()
{
    if (gwavi_close(m_gwavi) == -1)
    {
        qDebug("Error: call to gwavi_close() failed! %s\n", qPrintable(m_aviFilename));
        return __LINE__;
    }
    m_gwavi = 0;
    return 0;
}



