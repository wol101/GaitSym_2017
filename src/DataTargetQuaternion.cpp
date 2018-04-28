/*
 *  DataTargetQuaternion.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue July 14 2009.
 *  Copyright (c) 1009 Bill Sellers. All rights reserved.
 *
 */

#include <iostream>
#ifdef MALLOC_H_NEEDED
#include <malloc.h>
#endif
#ifdef ALLOCA_H_NEEDED
#include <alloca.h>
#endif

#include <ode/ode.h>

#include "DataTargetQuaternion.h"
#include "Body.h"
#include "PGDMath.h"
#include "Util.h"
#include "DataFile.h"
#include "Geom.h"

DataTargetQuaternion::DataTargetQuaternion()
{
    m_ValueList = 0;
    m_ValueListLength = -1;
}

DataTargetQuaternion::~DataTargetQuaternion()
{
    if (m_TargetTimeList) delete [] m_ValueList;
}

// note in this case the pointer is to a list of the elements of
// size quaternions
// note quaternion is (qs,qx,qy,qz)
void DataTargetQuaternion::SetTargetValues(int size, double *values)
{
    int i;
    if (size != m_TargetTimeListLength)
    {
        std::cerr << "DataTargetQuaternion::SetTargetValues error: size = " << size << "\n";
        return;
    }
    if (m_ValueListLength != size)
    {
        if (m_ValueList) delete [] m_ValueList;
        m_ValueListLength = size;
        m_ValueList = new pgd::Quaternion[m_ValueListLength];
    }
    for (i = 0 ; i < m_ValueListLength; i++)
    {
        m_ValueList[i].n = values[i * 4];
        m_ValueList[i].v.x = values[i * 4 + 1];
        m_ValueList[i].v.y = values[i * 4 + 2];
        m_ValueList[i].v.z = values[i * 4 + 3];
        m_ValueList[i].Normalize(); // always do this on input.
    }
}

// note in this case the pointer is to a string which is a list of the elements of
// size quaternions (or angle axis orientations following the standard d or r postscript
// convention
// note quaternion is (qs,qx,qy,qz)
void DataTargetQuaternion::SetTargetValues(const char *buf)
{
    int l = strlen(buf);
    char *lBuf = (char *)alloca((l + 1) * sizeof(char));
    char **lBufPtrs = (char **)alloca(l * sizeof(char *));
    dQuaternion quaternion;
    int i;

    strcpy(lBuf, buf);
    int size = DataFile::ReturnTokens(lBuf, lBufPtrs, l);

    if (size != m_TargetTimeListLength * 4)
    {
        std::cerr << "DataTargetQuaternion::SetTargetValues error: size = " << size << "\n";
        return;
    }

    if (m_ValueListLength != m_TargetTimeListLength)
    {
        if (m_ValueList) delete [] m_ValueList;
        m_ValueListLength = m_TargetTimeListLength;
        m_ValueList = new pgd::Quaternion[m_ValueListLength];
    }
    for (i = 0 ; i < m_ValueListLength; i++)
    {
        Util::GetQuaternion(&lBufPtrs[i * 4], quaternion);
        m_ValueList[i].n = quaternion[0];
        m_ValueList[i].v.x = quaternion[1];
        m_ValueList[i].v.y = quaternion[2];
        m_ValueList[i].v.z = quaternion[3];
    }
}

// returns the degree of match to the stored values
// in this case this is the angle between the two quaternions
double DataTargetQuaternion::GetError(int valueListIndex)
{
    const double *r;
    Body *body;
    Geom *geom;
    dQuaternion q;
    double angle = 0;
    if (valueListIndex < 0) valueListIndex = 0;
    if (valueListIndex >= m_ValueListLength)
    {
        std::cerr << "Warning: DataTargetQuaternion::GetMatchValue valueListIndex out of range\n";
        return 0;
    }

    if ((body = dynamic_cast<Body *>(m_Target)) != 0)
    {
        r = body->GetQuaternion();
        angle = pgd::FindAngle(m_ValueList[valueListIndex], pgd::Quaternion(r[0], r[1], r[2], r[3]));
    }
    else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
    {
        geom->GetWorldQuaternion(q);
        angle = pgd::FindAngle(m_ValueList[valueListIndex], pgd::Quaternion(q[0], q[1], q[2], q[3]));
    }
    else
    {
        std::cerr << "DataTargetQuaternion target missing error " << m_Name << "\n";
    }
    return angle;
}

// returns the degree of match to the stored values
// in this case this is the angle between the two quaternions
double DataTargetQuaternion::GetError(double time)
{
    const double *r;
    Body *body;
    Geom *geom;
    dQuaternion q;
    double angle = 0;

    int index = Util::BinarySearchRange(m_TargetTimeList, m_TargetTimeListLength, time);
    if (index < 0) index = 0;
    if (index >= m_ValueListLength - 1)
    {
        std::cerr << "Warning: DataTargetVector::GetMatchValue index out of range\n";
        return 0;
    }
    int indexNext = index + 1;

    // do a slerp interpolation between the target quaternions
    double interpolationFraction = (time - m_TargetTimeList[index]) / (m_TargetTimeList[indexNext] - m_TargetTimeList[index]);
    pgd::Quaternion interpolatedTarget = pgd::slerp(m_ValueList[index], m_ValueList[indexNext], interpolationFraction);

    if ((body = dynamic_cast<Body *>(m_Target)) != 0)
    {
        r = body->GetQuaternion();
        angle = pgd::FindAngle(interpolatedTarget, pgd::Quaternion(r[0], r[1], r[2], r[3]));
    }
    else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
    {
        geom->GetWorldQuaternion(q);
        angle = pgd::FindAngle(interpolatedTarget, pgd::Quaternion(q[0], q[1], q[2], q[3]));
    }
    else
    {
        std::cerr << "DataTargetQuaternion target missing error " << m_Name << "\n";
    }
    return angle;
}

void DataTargetQuaternion::Dump()
{
    if (m_Dump == false) return;

    if (m_FirstDump)
    {
        m_FirstDump = false;
        if (m_DumpStream == 0)
        {
            if (m_Name.size() == 0) std::cerr << "NamedObject::Dump error: can only dump a named object\n";
            std::string filename(m_Name);
            filename.append(".dump");
            m_DumpStream = new std::ofstream(filename.c_str());
            m_DumpStream->precision(17);
        }
        if (m_DumpStream)
        {
            *m_DumpStream << "Time\tTargetQW\tTargetQX\tTargetQY\tTargetQZ\tActualQW\tActualQX\tActualQY\tActualQZ\tAngle\n";
        }
    }

    Body *body;
    Geom *geom;
    const double *r;
    double angle = 0;
    dQuaternion q;

    if (m_DumpStream)
    {
        int valueListIndex = Util::BinarySearchRange(m_TargetTimeList, m_TargetTimeListLength, m_simulation->GetTime());
        if ((body = dynamic_cast<Body *>(m_Target)) != 0)
        {
            r = body->GetQuaternion();
            angle = pgd::FindAngle(m_ValueList[valueListIndex], pgd::Quaternion(r[0], r[1], r[2], r[3]));
        }
        else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
        {
            geom->GetWorldQuaternion(q);
            angle = pgd::FindAngle(m_ValueList[valueListIndex], pgd::Quaternion(q[0], q[1], q[2], q[3]));
        }

        *m_DumpStream << m_simulation->GetTime() <<
                "\t" << m_ValueList[valueListIndex].n << "\t" << m_ValueList[valueListIndex].v.x << "\t" << m_ValueList[valueListIndex].v.y << "\t" << m_ValueList[valueListIndex].v.z <<
                "\t" << q[0] << "\t" << q[1] << "\t" << q[2] << "\t" << q[3] <<
                "\t" << angle <<
                "\n";
    }
}

