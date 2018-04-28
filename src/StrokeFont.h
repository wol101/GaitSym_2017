#ifndef STROKEFONT_H
#define STROKEFONT_H

#include <ode/ode.h>

#ifdef USE_QT
class GLWidget;
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#endif

class StrokeFont
{
public:
    StrokeFont();
    ~StrokeFont();

    enum MarkerCode { XShape };

    void StrokeString(const char *string,    /* character string */
                      int length,            /* number of characters to draw */
                      float x,               /* x coordinate of bottom left of character */
                      float y,               /* y coordinate ... */
                      float cwidth,          /* cwidth of character cell */
                      float cheight,         /* cheight of character cell */
                      int xJustification,    /* 0 - left, 1 - centre, 2 - right */
                      int yJustification,    /* 0 - bottom, 1 - centre, 2 - top */
                      const float *matrix,
                      const float *translation);

    void StrokeCharacter(int ichar,            /* character code */
                         float x,              /* x coordinate of bottom left of character */
                         float y,              /* y coordinate ... */
                         float cwidth,         /* cwidth of character cell */
                         float cheight,         /* cheight of character cell */
                         const float *matrix,
                         const float *translation);

    void StrokeMarker(
                             MarkerCode code,       /* marker code */
                             float x,               /* x coordinate of centre of marker */
                             float y,               /* y coordinate ... */
                             float cwidth,          /* cwidth of character cell */
                             float cheight,         /* cheight of character cell */
                             const float *matrix,
                             const float *translation);

    void AddLine(float ix1, float iy1, float iz1, float ix2, float iy2, float iz2);
    void AddLine(float ix1, float iy1, float iz1, float ix2, float iy2, float iz2, const float *matrix, const float *translation);
    void StartLine() { m_start_line_flag = true; }
    void AddPoint(float ix1, float iy1, float iz1);
    void AddPoint(float ix1, float iy1, float iz1, const float *matrix, const float *translation);

    void AddCircle(float cx, float cy, float cz, float r, int num_segments);
    void AddArc(float cx, float cy, float cz, float r, float start_angle, float arc_angle, int num_segments);

    virtual void Draw();
    void SetDisplayPosition(double x, double y, double z);
    void SetDisplayRotation(const dMatrix3 R, bool fast = true);
    void SetDisplayRotationFromQuaternion(const dQuaternion q, bool fast = true);
    void SetDisplayRotationFromAxis(double x, double y, double z, bool fast = true);
    const double *GetDisplayPosition()  { return m_DisplayPosition; }
    const double *GetDisplayRotation()  { return m_DisplayRotation; }
    void Move(double x, double y, double z);
    void Scale(double x, double y, double z);

    void SetZ(float z) { m_z = z; }
    void SetRGBA(float r, float g, float b, float a) { m_r = r; m_g = g; m_b = b; m_a = a; }

    void ZeroLineBuffer() { n_lines = 0; }
    int GetNumLines() { return n_lines; }
    float *GetLineBuffer() { return line_buffer; }

#ifdef USE_QT
    GLWidget *glWidget() const;
    void setGlWidget(GLWidget *glWidget);
    QMatrix4x4 vpMatrix() const;
    void setVpMatrix(const QMatrix4x4 &vpMatrix);
    void Debug();
#endif


protected:
    float m_z;
    float m_r;
    float m_g;
    float m_b;
    float m_a;

    float m_last_x;
    float m_last_y;
    float m_last_z;
    bool m_start_line_flag;

    int n_lines;
    int max_lines;
    float *line_buffer;

    dVector3 m_DisplayPosition;
    dMatrix3 m_DisplayRotation;

#ifdef USE_QT
    GLWidget *m_glWidget;
    QOpenGLBuffer m_VBO;
    bool m_BufferObjectsAllocated;
    QMatrix4x4 m_vpMatrix;
#endif
};

#endif // STROKEFONT_H
