/*
 *  Util.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 12/07/2007.
 *  Copyright 2007 Bill Sellers. All rights reserved.
 *
 */

#if defined(_WIN32) || defined(WIN32)
#include <windows.h>
#endif

#include "Util.h"
#include "PGDMath.h"

#include <ode/ode.h>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <stdint.h>

#if !defined(_WIN32) && !defined(WIN32)
#include <sys/time.h>
#endif

// Euler decomposition examples from Geometric Tools for Computer Graphics, Scheider & Eberly 2003

// matrix format is:
// [r00 r01 r02]
// [r10 r11 r12]
// [r20 r21 r22]

// ode format definition
#define r00 mRot[0*4 + 0]
#define r01 mRot[0*4 + 1]
#define r02 mRot[0*4 + 2]
#define r10 mRot[1*4 + 0]
#define r11 mRot[1*4 + 1]
#define r12 mRot[1*4 + 2]
#define r20 mRot[2*4 + 0]
#define r21 mRot[2*4 + 1]
#define r22 mRot[2*4 + 2]

#define e11 mRot[0*4 + 0]
#define e12 mRot[0*4 + 1]
#define e13 mRot[0*4 + 2]
#define e21 mRot[1*4 + 0]
#define e22 mRot[1*4 + 1]
#define e23 mRot[1*4 + 2]
#define e31 mRot[2*4 + 0]
#define e32 mRot[2*4 + 1]
#define e33 mRot[2*4 + 2]

// these are intrinsic Euler angles (actually Tait-Bryan or Cardan angles according to Wikipedia)
// to do the extrinsic versions you just reverse the XYZ (so intrinsic XYZ = extrinsic ZYX)
// extrinsic is where the rotation axes are fixed
void Util::EulerDecompositionXYZ(const double *mRot, double& thetaX, double& thetaY, double& thetaZ)
{
    thetaY = asin(r02);
    if ( thetaY < M_PI/2 )
    {
        if ( thetaY > -M_PI/2 )
        {
            thetaX = atan2(-r12,r22);
            thetaZ = atan2(-r01,r00);
        }
        else
        {
            // not a unique solution
            thetaX = -atan2(r10,r11);
            thetaZ = 0;
        }
    }
    else
    {
        // not a unique solution
        thetaX = atan2(r10,r11);
        thetaZ = 0;
    }
}

void Util::EulerDecompositionXZY(const double *mRot, double& thetaX, double& thetaY, double& thetaZ)
{
    thetaZ = asin(-r01);
    if ( thetaZ < M_PI/2 )
    {
        if ( thetaZ > -M_PI/2 )
        {
            thetaX = atan2(r21,r11);
            thetaY = atan2(r02,r00);
        }
        else
        {
            // not a unique solution
            thetaX = -atan2(-r20,r22);
            thetaY = 0;
        }
    }
    else
    {
        // not a unique solution
        thetaX = atan2(-r20,r22);
        thetaY = 0;
    }
}

void Util::EulerDecompositionYXZ(const double *mRot, double& thetaX, double& thetaY, double& thetaZ)
{
    thetaX = asin(-r12);
    if ( thetaX < M_PI/2 )
    {
        if ( thetaX > -M_PI/2 )
        {
            thetaY = atan2(r02,r22);
            thetaZ = atan2(r10,r11);
        }
        else
        {
            // not a unique solution
            thetaY = -atan2(-r01,r00);
            thetaZ = 0;
        }
    }
    else
    {
        // not a unique solution
        thetaY = atan2(-r01,r00);
        thetaZ = 0;
    }
}

void Util::EulerDecompositionYZX(const double *mRot, double& thetaX, double& thetaY, double& thetaZ)
{
    thetaZ = asin(r10);
    if ( thetaZ < M_PI/2 )
    {
        if ( thetaZ > -M_PI/2 )
        {
            thetaY = atan2(-r20,r00);
            thetaX = atan2(-r12,r11);
        }
        else
        {
            // not a unique solution
            thetaY = -atan2(r21,r22);
            thetaX = 0;
        }
    }
    else
    {
        // not a unique solution
        thetaY = atan2(r21,r22);
        thetaX = 0;
    }
}

void Util::EulerDecompositionZXY(const double *mRot, double& thetaX, double& thetaY, double& thetaZ)
{
    thetaX = asin(r21);
    if ( thetaX < M_PI/2 )
    {
        if ( thetaX > -M_PI/2 )
        {
            thetaZ = atan2(-r01,r11);
            thetaY = atan2(-r20,r22);
        }
        else
        {
            // not a unique solution
            thetaZ = -atan2(r02,r00);
            thetaY = 0;
        }
    }
    else
    {
        // not a unique solution
        thetaZ = atan2(r02,r00);
        thetaY = 0;
    }
}

void Util::EulerDecompositionZYX(const double *mRot, double& thetaX, double& thetaY, double& thetaZ)
{
    thetaY = asin(-r20);
    if ( thetaY < M_PI/2 )
    {
        if ( thetaY > -M_PI/2 )
        {
            thetaZ = atan2(r10,r00);
            thetaX = atan2(r21,r22);
        }
        else
        {
            // not a unique solution
            thetaZ = -atan2(-r01,r02);
            thetaX = 0;
        }
    }
    else
    {
        // not a unique solution
        thetaZ = atan2(-r01,r02);
        thetaX = 0;
    }
}

// invert 3x3 matrix in ODE dMatrix3 format
void Util::Inverse(const double *mRot, dMatrix3 invMRot)
{
    double d = e11*e22*e33 -
    e11*e32*e23 +
    e21*e32*e13 -
    e21*e12*e33 +
    e31*e12*e23 -
    e31*e22*e13;

    if (d == 0) d = 1;

    invMRot[0] = (e22*e33-e23*e32)/d;
    invMRot[1] = -(e12*e33-e13*e32)/d;
    invMRot[2] = (e12*e23-e13*e22)/d;
    invMRot[4] = -(e21*e33-e23*e31)/d;
    invMRot[5] = (e11*e33-e13*e31)/d;
    invMRot[6] = -(e11*e23-e13*e21)/d;
    invMRot[8] = (e21*e32-e22*e31)/d;
    invMRot[9] = -(e11*e32-e12*e31)/d;
    invMRot[10] = (e11*e22-e12*e21)/d;
}

// finds the rotation matrix that will transform R1 to R2
// where R1 and R2 are both rotation matrices
void Util::FindRotation(const double *R1, const double *R2, dMatrix3 rotMat)
{
    // theory:
    // X.R1 = R2
    // X.R1.R1' = R2.R1'
    // X = R2.R1'
    // and that's what dMultiply2 does
    // as does dMULTIPLY2_333 which is quicker

    dMULTIPLY2_333(rotMat, R2, R1); // optimised version

}

// matrix format is:
// [r00 r01 r02]
// [r10 r11 r12]
// [r20 r21 r22]
void Util::DumpMatrix(const double *mRot)
{
    fprintf(stderr, "[%9.4f %9.4f %9.4f]\n", r00, r01, r02);
    fprintf(stderr, "[%9.4f %9.4f %9.4f]\n", r10, r11, r12);
    fprintf(stderr, "[%9.4f %9.4f %9.4f]\n", r20, r21, r22);
    fprintf(stderr, "\n");
}

void Util::Tokenizer(const char *constbuf, std::vector<std::string> &tokens, const char *stopList)
{
    char *localBuf = (char *)malloc(strlen(constbuf) + 1);
    char *ptr = localBuf;
    strcpy(ptr, constbuf);
    char *qp;
    char byte;
    char oneChar[2] = {0, 0};
    if (stopList == 0) stopList = "{};,=:&|!()+-/*[]'<>^";

     while (*ptr)
     {
         // find non-whitespace
         if (*ptr < 33)
         {
             ptr++;
             continue;
         }

         // is it in stoplist
         if (strchr(stopList, *ptr))
         {
             oneChar[0] = *ptr;
             tokens.push_back(oneChar);
             ptr++;
             continue;
         }

         // is it a double quote?
         if (*ptr == '"')
         {
             ptr++;
             qp = strchr(ptr, '"');
             if (qp)
             {
                 *qp = 0;
                 tokens.push_back(ptr);
                 *qp = '"';
                 ptr = qp + 1;
                 continue;
             }
         }

         qp = ptr;
         while (*qp >= 33 && strchr(stopList, *qp) == 0 && *qp != '"')
         {
             qp++;
         }
         byte = *qp;
         *qp = 0;
         tokens.push_back(ptr);
         if (byte == 0) break;
         *qp = byte;
         ptr = qp;
     }

     free(localBuf);
}

// function to identify and if necessary convert (angle, axisx, axisy, axiz) representations
// to (qs,qx,qy,qz) quaternions
// angle is identified by a postscript r for radians and d for degrees
// no postscript means that the value is already a quaternion
double *Util::GetQuaternion(char *bufPtrs[], double *q)
{
    int i;
    for (i = 0; i < 4; i++) q[i] = strtod(bufPtrs[i], 0);

    char *p;
    p = bufPtrs[0];
    while (*p) p++;
    p--; // pointing to last character of the string

    if (*p == 'r') // radian angle axis
    {
        pgd::Quaternion qq = pgd::MakeQFromAxis(q[1], q[2], q[3], q[0]);
        q[0] = qq.n;
        q[1] = qq.v.x;
        q[2] = qq.v.y;
        q[3] = qq.v.z;
        return q;
    }
    if (*p == 'd') // degee angle axis
    {
        pgd::Quaternion qq = pgd::MakeQFromAxis(q[1], q[2], q[3], pgd::DegreesToRadians(q[0]));
        q[0] = qq.n;
        q[1] = qq.v.x;
        q[2] = qq.v.y;
        q[3] = qq.v.z;
        return q;
    }
    return q;
}

// function to return an angle from a string
// angle is identified by a postscript r for radians and d for degrees
// no postscript means that the value is already in radians
double Util::GetAngle(const char *buf)
{
    double angle = strtod(buf, 0);

    const char *p;
    p = buf;
    while (*p) p++;
    p--; // pointing to last character of the string

    if (*p == 'r') // radian angle
    {
        return angle;
    }
    if (*p == 'd') // degee angle
    {
        return angle * M_PI / 180;
    }
    return angle;
}

// function to return the distance between two lines
// p1 - point on line 1
// d1 - direction of line 1
// p2 - point on line 2
// d2 - direction of line 2
double Util::DistanceBetweenTwoLines(pgd::Vector p1, pgd::Vector d1, pgd::Vector p2, pgd::Vector d2)
{
    // first find ther perpendicular to the two vectors
    pgd::Vector perpendicular = d1 ^ d2;
    perpendicular.Normalize();

    // now find a vector from l1 to l2
    pgd::Vector link = p2 - p1;

    // shortest distance is the component of this vector in the direction of the normal
    double distance = link * perpendicular;

    return distance;
}

/*
   Calculate the line segment PaPb that is the shortest route between
   two lines P1P2 and P3P4. Calculate also the values of mua and mub where
      Pa = P1 + mua (P2 - P1)
      Pb = P3 + mub (P4 - P3)
   Return false if no solution exists.
*/
bool Util::LineLineIntersect(pgd::Vector p1, pgd::Vector p2,
                            pgd::Vector p3, pgd::Vector p4,
                            pgd::Vector *pa, pgd::Vector *pb,
                            double *mua, double *mub)
{
    pgd::Vector p13,p43,p21;
    double d1343,d4321,d1321,d4343,d2121;
    double numer,denom;
    const double EPS = pgd::epsilon;

    p13.x = p1.x - p3.x;
    p13.y = p1.y - p3.y;
    p13.z = p1.z - p3.z;
    p43.x = p4.x - p3.x;
    p43.y = p4.y - p3.y;
    p43.z = p4.z - p3.z;
    if (ABS(p43.x)  < EPS && ABS(p43.y)  < EPS && ABS(p43.z)  < EPS)
        return(false);
    p21.x = p2.x - p1.x;
    p21.y = p2.y - p1.y;
    p21.z = p2.z - p1.z;
    if (ABS(p21.x)  < EPS && ABS(p21.y)  < EPS && ABS(p21.z)  < EPS)
        return(false);

    d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
    d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
    d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
    d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
    d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

    denom = d2121 * d4343 - d4321 * d4321;
    if (ABS(denom) < EPS)
        return(false);
    numer = d1343 * d4321 - d1321 * d4343;

    *mua = numer / denom;
    *mub = (d1343 + d4321 * (*mua)) / d4343;

    pa->x = p1.x + *mua * p21.x;
    pa->y = p1.y + *mua * p21.y;
    pa->z = p1.z + *mua * p21.z;
    pb->x = p3.x + *mub * p43.x;
    pb->y = p3.y + *mub * p43.y;
    pb->z = p3.z + *mub * p43.z;

    return(true);
}

// this routine converts a text string to a bitmap
// the text string ignores whitespace and the zero character
// is defined. Anything not whitespace and not the zero character
// is a set bit. It returns a dynamically allocated character array
// of 0 and 1. Optionally the Y can be reversed since by default the
// origin is the top left and often the bottom right is what is wanted
unsigned char *Util::AsciiToBitMap(const char *string, int width, int height, char setChar, bool reverseY)
{
    const char *pin = string;
    int outputLen = width * height;
    unsigned char *bitmap = new unsigned char[outputLen];
    unsigned char *pout;
    int j;

    // memset(bitmap, 3, outputLen);

    if (reverseY == false)
    {
        pout = bitmap;
        while (outputLen > 0)
        {
            if (*pin > 32)
            {
                if (*pin == setChar) *pout = 1;
                else *pout = 0;
                pout++;
                outputLen--;
            }
            pin++;
        }
    }
    else
    {
        pout = bitmap + outputLen - width;
        j = 0;
        while (outputLen > 0)
        {
            if (*pin > 32)
            {
                if (*pin == setChar) *pout = 1;
                else *pout = 0;
                pout++;
                j++;
                if (j >= width)
                {
                    j = 0;
                    pout -= (2 * width);
                }
                outputLen--;
            }
            pin++;
        }
    }

#if 0
    for (int i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            std::cerr << int(bitmap[j + i * height]);
        }
        std::cerr << "\n";
    }
#endif

    return bitmap;
}

void Util::FindAndReplace( std::string *source, const std::string &find, const std::string &replace )
{
    std::string::size_type j;
    while ((j = source->find( find )) != std::string::npos)
    {
        source->replace( j, find.length(), replace );
    }
}

// finds the indices of the members in a sorted list that bracket a particular value
// intially *lowBound should equal zero, and *highBound should equal the length of the list
void Util::FindBoundsCheck(double *list, double x, int *lowBound, int *highBound)
{
    if (x < list[*lowBound])
    {
        *lowBound = *lowBound - 1;
        *highBound = *lowBound;
    }
    if (x > list[*highBound])
    {
        *lowBound = *highBound;
        *highBound = *highBound + 1;
    }
    FindBounds(list, x, lowBound, highBound);
}

void Util::FindBounds(double *list, double x, int *lowBound, int *highBound)
{
    if ((*highBound - *lowBound) <= 1) return; // end condition
    int pivotIndex = (*lowBound + *highBound) / 2;
    double pivot = list[pivotIndex];
    double low = list[*lowBound];
    double high = list[*highBound];
    if (x >= low && x <= pivot) *highBound = pivotIndex;
    else if (x >= pivot && x <= high) *lowBound = pivotIndex;
    FindBounds(list, x, lowBound, highBound);
}

double Util::GetTime()
{
#if defined(_WIN32) || defined(WIN32)
    FILETIME ft;
    unsigned __int64 tmpres = 0; // this is count in 100 nanoseconds intervals
    GetSystemTimeAsFileTime(&ft);

    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;

    return ((double)tmpres) / 10000000.0;
#else
    struct timeval tv ;
    gettimeofday(&tv, 0);
    return ((double)tv.tv_sec) + ((double)tv.tv_usec) / 1000000.0;
#endif
}

int Util::QuickInt(const char *p)
{
    int x = 0;
    bool neg = false;
    if (*p == '-') {
        neg = true;
        ++p;
    }
    while (*p >= '0' && *p <= '9')
    {
        x = (x*10) + (*p - '0');
        ++p;
    }
    if (neg)
    {
        x = -x;
    }
    return x;
}

double Util::QuickDouble(const char *p)
{
    //double t = atof(p);
    double r = 0.0;
    bool neg = false;
    if (*p == '-')
    {
        neg = true;
        ++p;
    }
    while (*p >= '0' && *p <= '9')
    {
        r = (r*10.0) + (*p - '0');
        ++p;
    }
    if (*p == '.')
    {
        double f = 0.0;
        int n = 0;
        ++p;
        while (*p >= '0' && *p <= '9')
        {
            f = (f*10.0) + (*p - '0');
            ++p;
            ++n;
        }
        r += f / QuickPow(10.0, n);
    }
    if (*p == 'e' || *p == 'E')
    {
        ++p;
        int m = QuickInt(p);
        r *= QuickPow(10.0, m);
    }
    if (neg)
    {
        r = -r;
    }
    //if (fabs(t - r) > 0.01)
    //    std::cerr << "t = " << t << " r = " << r << "\n";
    return r;
}

double Util::QuickPow(double base, int exp)
{
    if (exp >= 0)
    {
        double result = 1;
        while (exp)
        {
            if (exp & 1)
                result *= base;
            exp >>= 1;
            base *= base;
        }

        return result;
    }
    else
    {
        exp = -exp;
        double result = 1;
        while (exp)
        {
            if (exp & 1)
                result *= base;
            exp >>= 1;
            base *= base;
        }

        return 1.0 / result;
    }
}

