#ifndef AVIWRITER_H
#define AVIWRITER_H

#include <QString>

class AVIWriter
{
public:
    AVIWriter();
    ~AVIWriter();
    int InitialiseFile(const QString &aviFilename, unsigned int width, unsigned int height, unsigned int fps);
    int WriteAVI(unsigned int width, unsigned int height, const unsigned char *rgb, int quality);

protected:
    int CloseFile();

    struct gwavi_t *m_gwavi;
    unsigned int m_width;
    unsigned int m_height;
    unsigned int m_fps;
    QString m_aviFilename;
};

#endif // AVIWRITER_H
