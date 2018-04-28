/*
 *  Util.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Sat Dec 06 2003.
 *  Copyright (c) 2003 Bill Sellers. All rights reserved.
 *
 *  All the routines I can't think of a better place for
 *
 */

#ifndef __UTIL_H__
#define __UTIL_H__

#include "PGDMath.h"

#include <ode/ode.h>

#include <cmath>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <stdint.h>

#if defined(_WIN32) || defined(WIN32)
#define strcasecmp(s1, s2) _stricmp(s1, s2)
#else
#include <strings.h>
#endif

#define THROWIFZERO(a) if ((a) == 0) throw __LINE__
#define THROWIF(a) if ((a) != 0) throw __LINE__
#define SQUARE(a) ((a) * (a))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define ABS(a) ((a) >= 0 ? (a) : -(a))
#define ODD(n) ((n) & 1)
#define SWAP(a,b) { (a) = (a)+(b); (b) = (a)-(b); (a) = (a)-(b); }

class Util {
public:

    // calculate cross product (vector product)
    inline static void CrossProduct3x1(const double *a, const double *b, double *c)
{
        c[0] = a[1] * b[2] - a[2] * b[1];
        c[1] = a[2] * b[0] - a[0] * b[2];
        c[2] = a[0] * b[1] - a[1] * b[0];
};

// calculate dot product (scalar product)
inline static double DotProduct3x1(const double *a, const double *b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
};

// calculate length of vector
inline static double Magnitude3x1(const double *a)
{
    return sqrt(SQUARE(a[0]) + SQUARE(a[1]) + SQUARE(a[2]));
};

// calculate distance between two points
inline static double Distance3x1(const double *a, const double *b)
{
    return sqrt(SQUARE(a[0] - b[0]) + SQUARE(a[1] - b[1]) + SQUARE(a[2] - b[2]));
};

// calculate unit vector
inline static void Unit3x1(double *a)
{
    double len = sqrt(SQUARE(a[0]) + SQUARE(a[1]) + SQUARE(a[2]));
    // default fixup for zero length vectors
    if (ABS(len) < 1e-30)
    {
        a[0] = 1;
        a[1] = 0;
        a[2] = 0;
    }
    else
    {
        a[0] /= len;
        a[1] /= len;
        a[2] /= len;
    }
};

// c = a + b vectors
inline static void Add3x1(const double *a, const double *b, double *c)
{
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
};

// c = a - b vectors
inline static void Subtract3x1(const double *a, const double *b, double *c)
{
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
};

// c = scalar * a
inline static void ScalarMultiply3x1(const double scalar, const double *a, double *c)
{
    c[0] = a[0] * scalar;
    c[1] = a[1] * scalar;
    c[2] = a[2] * scalar;
};

// b = a
inline static void Copy3x1(const double *a, double *b)
{
    b[0] = a[0];
    b[1] = a[1];
    b[2] = a[2];
};


inline static void ZPlaneRotate(double theta,
                                double *location)
{
    double internal[3];

    // get a local copy

    internal[0] = location[0];
    internal[1] = location[1];
    // internal[2] = location[2];

    // rotation code

    double ctheta = cos(theta);
    double stheta = sin(theta);

    // z planar rotation:
    location[0] = internal[0]*ctheta - internal[1]*stheta;
    location[1] = internal[1]*ctheta + internal[0]*stheta;
};

inline static bool OutRange(double v, double l, double h)
{
    if (v < l)
        return true;
    if (v > h)
        return true;
    return false;
}

inline static double Double(const char *buf)
{
    return strtod(buf, 0);
}

inline static double Double(const unsigned char *buf)
{
    return strtod((char *)buf, 0);
}

inline static void Double(const char *buf, int n, double *d)
{
    const char *cptr = buf;
    char *ptr;
    for (int i = 0; i < n; i++)
    {
        d[i] = strtod(cptr, &ptr);
        cptr = ptr;
    }
}

inline static void Double(const unsigned char *buf, int n, double *d)
{
    const char *cptr = (const char *)buf;
    char *ptr;
    for (int i = 0; i < n; i++)
    {
        d[i] = strtod(cptr, &ptr);
        cptr = ptr;
    }
}

inline static int Int(const char *buf)
{
    return (int)strtol(buf, 0, 10);
}

inline static int Int(const unsigned char *buf)
{
    return (int)strtol((char *)buf, 0, 10);
}

inline static void Int(const char *buf, int n, int *d)
{
    const char *cptr = buf;
    char *ptr;
    for (int i = 0; i < n; i++)
    {
        d[i] = (int)strtol(cptr, &ptr, 10);
        cptr = ptr;
    }
}

inline static void Int(unsigned char *buf, int n, int *d)
{
    const char *cptr = (const char *)buf;
    char *ptr;
    for (int i = 0; i < n; i++)
    {
        d[i] = (int)strtol(cptr, &ptr, 10);
        cptr = ptr;
    }
}

// Note modifies string
inline static bool Bool(char *buf)
{
    Strip(buf);
    if (strcasecmp(buf, "false") == 0) return false;
    if (strcasecmp(buf, "true") == 0) return true;
    if (strtol(buf, 0, 10) != 0) return true;
    return false;
}

// Note modifies string
inline static bool Bool(unsigned char *buf)
{
    Strip((char *)buf);
    if (strcasecmp((char *)buf, "false") == 0) return false;
    if (strcasecmp((char *)buf, "true") == 0) return true;
    if (strtol((char *)buf, 0, 10) != 0) return true;
    return false;
}

inline static bool Bool(const char *cbuf)
{
    char buf[16];
    strncpy(buf, cbuf, 16);
    Strip(buf);
    if (strcasecmp(buf, "false") == 0) return false;
    if (strcasecmp(buf, "true") == 0) return true;
    if (strtol(buf, 0, 10) != 0) return true;
    return false;
}

inline static bool Bool(const unsigned char *cbuf)
{
    char buf[16];
    strncpy(buf, (const char *)cbuf, 16);
    Strip(buf);
    if (strcasecmp(buf, "false") == 0) return false;
    if (strcasecmp(buf, "true") == 0) return true;
    if (strtol(buf, 0, 10) != 0) return true;
    return false;
}

// strip out beginning and ending whitespace
// Note modifies string
inline static void Strip(char *str)
{
    char *p1, *p2;

    if (*str == 0) return;

    // heading whitespace
    if (*str <= ' ')
    {
        p1 = str;
        while (*p1)
        {
            if (*p1 > ' ') break;
            p1++;
        }
        p2 = str;
        while (*p1)
        {
            *p2 = *p1;
            p1++;
            p2++;
        }
        *p2 = 0;
    }

    if (*str == 0) return;

    // tailing whitespace
    p1 = str;
    while (*p1)
    {
        p1++;
    }
    p1--;
    while (*p1 <= ' ')
    {
        p1--;
    }
    p1++;
    *p1 = 0;

    return;
}

// Count whitespace delimited tokens (tokens surrounded by double quotes are considered a single token)
inline static int CountTokens(const char *string)
{
    const char *p = string;
    bool inToken = false;
    int count = 0;

    while (*p != 0)
    {
        if (inToken == false && *p > 32)
        {
            inToken = true;
            count++;
            if (*p == '"')
            {
                p++;
                while (*p != '"')
                {
                    p++;
                    if (*p == 0) return count;
                }
            }
        }
        else if (inToken == true && *p <= 32)
        {
            inToken = false;
        }
        p++;
    }
    return count;
}
inline static int CountTokens(const unsigned char *string) { return CountTokens((const char *)string); }

// linear interpolate using 2 sets of (x,y) coordinates to define the line
inline static double Interpolate(double x1, double y1, double x2, double y2, double x)
{
    // y - y1 = ( (y2 - y1) / (x2 - x1) ) * (x - x1)
    double y =  ( ( (y2 - y1) / (x2 - x1) ) * (x - x1) ) + y1;
    return y;
}

// return the index of a matching item in a sorted array
template <class T> inline static int BinarySearchMatch
(T array[ ], int listlen, T item)
{
    int first = 0;
    int last = listlen-1;
    int mid;
    while (first <= last)
    {
        mid = (first + last) / 2;
        if (array[mid] < item) first = mid + 1;
        else if (array[mid] > item) last = mid - 1;
        else return mid;
    }

    return -1;
}

// return the index of a matching item in a sorted array
// special case when I'm searching for a range rather than an exact match
// returns the index of array[index] <= item < array[index+1]
template <class T> inline static int BinarySearchRange
(T array[ ], int listlen, T item)
{
    int first = 0;
    int last = listlen-1;
    int mid;
    while (first <= last)
    {
        mid = (first + last) / 2;
        if (array[mid + 1] <= item) first = mid + 1;
        else if (array[mid] > item) last = mid - 1;
        else return mid;
    }
    return -1;
}

static void EulerDecompositionXYZ(const double *mRot, double& thetaX, double& thetaY, double& thetaZ);
static void EulerDecompositionXZY(const double *mRot, double& thetaX, double& thetaY, double& thetaZ);
static void EulerDecompositionYXZ(const double *mRot, double& thetaX, double& thetaY, double& thetaZ);
static void EulerDecompositionYZX(const double *mRot, double& thetaX, double& thetaY, double& thetaZ);
static void EulerDecompositionZXY(const double *mRot, double& thetaX, double& thetaY, double& thetaZ);
static void EulerDecompositionZYX(const double *mRot, double& thetaX, double& thetaY, double& thetaZ);

static void Inverse(const double *mRot, dMatrix3 invMRot);
static void FindRotation(const double *R1, const double *R2, dMatrix3 rotMat);
static void DumpMatrix(const double *mRot);

static void Tokenizer(const char *constbuf, std::vector<std::string> &tokens, const char *stopList);

static double *GetQuaternion(char *bufPtrs[], double *q);
static double GetAngle(const char *buf);

static double DistanceBetweenTwoLines(pgd::Vector p1, pgd::Vector d1, pgd::Vector p2, pgd::Vector d2);
static bool LineLineIntersect(pgd::Vector p1, pgd::Vector p2,
                       pgd::Vector p3, pgd::Vector p4,
                       pgd::Vector *pa, pgd::Vector *pb,
                       double *mua, double *mub);

static unsigned char *AsciiToBitMap(const char *string, int width, int height, char setChar, bool reverseY = false);
static void FindAndReplace( std::string *source, const std::string &find, const std::string &replace );
static void FindBoundsCheck(double *list, double x, int *lowBound, int *highBound); // might be quicker than BinarySearchRange for special case
static void FindBounds(double *list, double x, int *lowBound, int *highBound);
static double GetTime();
static int QuickInt(const char *p);
static double QuickDouble(const char *p);
static double QuickPow(double base, int exp);


static void BinaryOutput(std::ostream &stream, int8_t v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, uint8_t v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, int16_t v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, uint16_t v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, int32_t v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, uint32_t v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, int64_t v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, uint64_t v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, bool v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, float v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, double v) { stream.write((const char *)&v, sizeof(v)); }
static void BinaryOutput(std::ostream &stream, const std::string &v) { BinaryOutput(stream, (uint32_t)v.size()); stream.write((const char *)v.c_str(), v.size()); }

};

#endif                   // __UTIL_H__
