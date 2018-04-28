/*
 *  DataTargetScalar.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on Tue July 14 2009.
 *  Copyright (c) 1009 Bill Sellers. All rights reserved.
 *
 */

#include <iostream>

#include <ode/ode.h>

#include "DataTargetScalar.h"
#include "Body.h"
#include "HingeJoint.h"
#include "BallJoint.h"
#include "UniversalJoint.h"
#include "PositionReporter.h"
#include "Geom.h"
#include "Util.h"
#include "TegotaeDriver.h"

DataTargetScalar::DataTargetScalar()
{
    m_DataType = XP;
}

DataTargetScalar::~DataTargetScalar()
{
    if (m_TargetTimeList) delete [] m_ValueList;
}

// returns the difference between the target actual value and the desired value (actual - desired)
double DataTargetScalar::GetError(int index)
{
    double errorScore = 0;
    const double *r;
    dVector3 result;
    dQuaternion q;
    pgd::Quaternion pq;
    pgd::Vector pv;

    Body *body;
    HingeJoint *hingeJoint;
    BallJoint *ballJoint;
    UniversalJoint *universalJoint;
    PositionReporter *positionReporter;
    Geom *geom;
    TegotaeDriver *tegotaeDriver;

    if (index < 0) index = 0;
    if (index >= m_ValueListLength)
    {
        std::cerr << "Warning: DataTargetScalar::GetMatchValue index out of range\n";
        return 0;
    }

    if ((body = dynamic_cast<Body *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case Q0:
            r = body->GetQuaternion();
            errorScore = (r[0] - m_ValueList[index]);
            break;
        case Q1:
            r = body->GetQuaternion();
            errorScore = (r[1] - m_ValueList[index]);
            break;
        case Q2:
            r = body->GetQuaternion();
            errorScore = (r[2] - m_ValueList[index]);
            break;
        case Q3:
            r = body->GetQuaternion();
            errorScore = (r[3] - m_ValueList[index]);
            break;
        case XP:
            r = body->GetPosition();
            errorScore = (r[0] - m_ValueList[index]);
            break;
        case YP:
            r = body->GetPosition();
            errorScore = (r[1] - m_ValueList[index]);
            break;
        case ZP:
            r = body->GetPosition();
            errorScore = (r[2] - m_ValueList[index]);
            break;
        case XV:
            r = body->GetLinearVelocity();
            errorScore = (r[0] - m_ValueList[index]);
            break;
        case YV:
            r = body->GetLinearVelocity();
            errorScore = (r[1] - m_ValueList[index]);
            break;
        case ZV:
            r = body->GetLinearVelocity();
            errorScore = (r[2] - m_ValueList[index]);
            break;
        case XRV:
            r = body->GetAngularVelocity();
            errorScore = (r[0] - m_ValueList[index]);
            break;
        case YRV:
            r = body->GetAngularVelocity();
            errorScore = (r[1] - m_ValueList[index]);
            break;
        case ZRV:
            r = body->GetAngularVelocity();
            errorScore = (r[2] - m_ValueList[index]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((positionReporter = dynamic_cast<PositionReporter *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case Q0:
            pq = positionReporter->GetWorldQuaternion();
            errorScore = (pq.n - m_ValueList[index]);
            break;
        case Q1:
            pq = positionReporter->GetWorldQuaternion();
            errorScore = (pq.v.x - m_ValueList[index]);
            break;
        case Q2:
            pq = positionReporter->GetWorldQuaternion();
            errorScore = (pq.v.y - m_ValueList[index]);
            break;
        case Q3:
            pq = positionReporter->GetWorldQuaternion();
            errorScore = (pq.v.z - m_ValueList[index]);
            break;
        case XP:
            pv = positionReporter->GetWorldPosition();
            errorScore = (pv.x - m_ValueList[index]);
            break;
        case YP:
            pv = positionReporter->GetWorldPosition();
            errorScore = (pv.y - m_ValueList[index]);
            break;
        case ZP:
            pv = positionReporter->GetWorldPosition();
            errorScore = (pv.z - m_ValueList[index]);
            break;
        case XV:
            pv = positionReporter->GetWorldVelocity();
            errorScore = (pv.x - m_ValueList[index]);
            break;
        case YV:
            pv = positionReporter->GetWorldVelocity();
            errorScore = (pv.y - m_ValueList[index]);
            break;
        case ZV:
            pv = positionReporter->GetWorldVelocity();
            errorScore = (pv.z - m_ValueList[index]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((hingeJoint = dynamic_cast<HingeJoint *>(m_Target)) != 0)
    {
        hingeJoint->GetHingeAnchor(result);
        switch (m_DataType)
        {
        case XP:
            errorScore = (result[0] - m_ValueList[index]);
            break;
        case YP:
            errorScore = (result[1] - m_ValueList[index]);
            break;
        case ZP:
            errorScore = (result[2] - m_ValueList[index]);
            break;
        case Angle:
            errorScore = (hingeJoint->GetHingeAngle() - m_ValueList[index]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((ballJoint = dynamic_cast<BallJoint *>(m_Target)) != 0)
    {
        ballJoint->GetBallAnchor(result);
        switch (m_DataType)
        {
        case XP:
            errorScore = (result[0] - m_ValueList[index]);
            break;
        case YP:
            errorScore = (result[1] - m_ValueList[index]);
            break;
        case ZP:
            errorScore = (result[2] - m_ValueList[index]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((universalJoint = dynamic_cast<UniversalJoint *>(m_Target)) != 0)
    {
        universalJoint->GetUniversalAnchor(result);
        switch (m_DataType)
        {
        case XP:
            errorScore = (result[0] - m_ValueList[index]);
            break;
        case YP:
            errorScore = (result[1] - m_ValueList[index]);
            break;
        case ZP:
            errorScore = (result[2] - m_ValueList[index]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case Q0:
            geom->GetWorldQuaternion(q);
            errorScore = (q[0] - m_ValueList[index]);
            break;
        case Q1:
            geom->GetWorldQuaternion(q);
            errorScore = (q[1] - m_ValueList[index]);
            break;
        case Q2:
            geom->GetWorldQuaternion(q);
            errorScore = (q[2] - m_ValueList[index]);
            break;
        case Q3:
            geom->GetWorldQuaternion(q);
            errorScore = (q[3] - m_ValueList[index]);
            break;
        case XP:
            geom->GetWorldPosition(result);
            errorScore = (result[0] - m_ValueList[index]);
            break;
        case YP:
            geom->GetWorldPosition(result);
            errorScore = (result[1] - m_ValueList[index]);
            break;
        case ZP:
            geom->GetWorldPosition(result);
            errorScore = (result[2] - m_ValueList[index]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case Q0:
            geom->GetWorldQuaternion(q);
            errorScore = (q[0] - m_ValueList[index]);
            break;
        case Q1:
            geom->GetWorldQuaternion(q);
            errorScore = (q[1] - m_ValueList[index]);
            break;
        case Q2:
            geom->GetWorldQuaternion(q);
            errorScore = (q[2] - m_ValueList[index]);
            break;
        case Q3:
            geom->GetWorldQuaternion(q);
            errorScore = (q[3] - m_ValueList[index]);
            break;
        case XP:
            geom->GetWorldPosition(result);
            errorScore = (result[0] - m_ValueList[index]);
            break;
        case YP:
            geom->GetWorldPosition(result);
            errorScore = (result[1] - m_ValueList[index]);
            break;
        case ZP:
            geom->GetWorldPosition(result);
            errorScore = (result[2] - m_ValueList[index]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((tegotaeDriver = dynamic_cast<TegotaeDriver *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case DriverError:
            double X = tegotaeDriver->X();
            double Y = tegotaeDriver->Y();
            errorScore = sqrt(X*X + Y*Y) - m_ValueList[index];
            break;
        }
    }
    else if (m_Target == 0)
    {
        switch(m_DataType)
        {
        case MetabolicEnergy:
            errorScore = (m_simulation->GetMetabolicEnergy() - m_ValueList[index]);
            break;
        case MechanicalEnergy:
            errorScore = (m_simulation->GetMechanicalEnergy() - m_ValueList[index]);
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else
    {
        std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataTarget " << m_DataType << "\n";
    }

    return errorScore;
}
// returns the difference between the target actual value and the desired value (actual - desired)
double DataTargetScalar::GetError(double time)
{
    double errorScore = 0;
    const double *r;
    dVector3 result;
    dQuaternion q;
    pgd::Quaternion pq;
    pgd::Vector pv;

    Body *body;
    HingeJoint *hingeJoint;
    BallJoint *ballJoint;
    UniversalJoint *universalJoint;
    PositionReporter *positionReporter;
    Geom *geom;
    TegotaeDriver *tegotaeDriver;

    int index = Util::BinarySearchRange(m_TargetTimeList, m_TargetTimeListLength, time);
    if (index < 0) index = 0;
    if (index >= m_ValueListLength - 1)
    {
        std::cerr << "Warning: DataTargetScalar::GetMatchValue index out of range\n";
        return 0;
    }
    int indexNext = index + 1;

    if ((body = dynamic_cast<Body *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case Q0:
            r = body->GetQuaternion();
            errorScore = (r[0] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case Q1:
            r = body->GetQuaternion();
            errorScore = (r[1] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case Q2:
            r = body->GetQuaternion();
            errorScore = (r[2] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case Q3:
            r = body->GetQuaternion();
            errorScore = (r[3] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case XP:
            r = body->GetPosition();
            errorScore = (r[0] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case YP:
            r = body->GetPosition();
            errorScore = (r[1] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case ZP:
            r = body->GetPosition();
            errorScore = (r[2] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case XV:
            r = body->GetLinearVelocity();
            errorScore = (r[0] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case YV:
            r = body->GetLinearVelocity();
            errorScore = (r[1] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case ZV:
            r = body->GetLinearVelocity();
            errorScore = (r[2] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case XRV:
            r = body->GetAngularVelocity();
            errorScore = (r[0] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case YRV:
            r = body->GetAngularVelocity();
            errorScore = (r[1] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case ZRV:
            r = body->GetAngularVelocity();
            errorScore = (r[2] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((positionReporter = dynamic_cast<PositionReporter *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case Q0:
            pq = positionReporter->GetWorldQuaternion();
            errorScore = (pq.n - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case Q1:
            pq = positionReporter->GetWorldQuaternion();
            errorScore = (pq.v.x - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case Q2:
            pq = positionReporter->GetWorldQuaternion();
            errorScore = (pq.v.y - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case Q3:
            pq = positionReporter->GetWorldQuaternion();
            errorScore = (pq.v.z - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case XP:
            pv = positionReporter->GetWorldPosition();
            errorScore = (pv.x - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case YP:
            pv = positionReporter->GetWorldPosition();
            errorScore = (pv.y - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case ZP:
            pv = positionReporter->GetWorldPosition();
            errorScore = (pv.z - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case XV:
            pv = positionReporter->GetWorldVelocity();
            errorScore = (pv.x - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case YV:
            pv = positionReporter->GetWorldVelocity();
            errorScore = (pv.y - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case ZV:
            pv = positionReporter->GetWorldVelocity();
            errorScore = (pv.z - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((hingeJoint = dynamic_cast<HingeJoint *>(m_Target)) != 0)
    {
        hingeJoint->GetHingeAnchor(result);
        switch (m_DataType)
        {
        case XP:
            errorScore = (result[0] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case YP:
            errorScore = (result[1] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case ZP:
            errorScore = (result[2] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case Angle:
            errorScore = (hingeJoint->GetHingeAngle() - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((ballJoint = dynamic_cast<BallJoint *>(m_Target)) != 0)
    {
        ballJoint->GetBallAnchor(result);
        switch (m_DataType)
        {
        case XP:
            errorScore = (result[0] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case YP:
            errorScore = (result[1] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case ZP:
            errorScore = (result[2] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((universalJoint = dynamic_cast<UniversalJoint *>(m_Target)) != 0)
    {
        universalJoint->GetUniversalAnchor(result);
        switch (m_DataType)
        {
        case XP:
            errorScore = (result[0] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case YP:
            errorScore = (result[1] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case ZP:
            errorScore = (result[2] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case Q0:
            geom->GetWorldQuaternion(q);
            errorScore = (q[0] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case Q1:
            geom->GetWorldQuaternion(q);
            errorScore = (q[1] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case Q2:
            geom->GetWorldQuaternion(q);
            errorScore = (q[2] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case Q3:
            geom->GetWorldQuaternion(q);
            errorScore = (q[3] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case XP:
            geom->GetWorldPosition(result);
            errorScore = (result[0] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case YP:
            geom->GetWorldPosition(result);
            errorScore = (result[1] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case ZP:
            geom->GetWorldPosition(result);
            errorScore = (result[2] - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else if ((tegotaeDriver = dynamic_cast<TegotaeDriver *>(m_Target)) != 0)
    {
        switch (m_DataType)
        {
        case DriverError:
            errorScore = tegotaeDriver->GetValue(time) - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time);
        }
    }
    else if (m_Target == 0)
    {
        switch(m_DataType)
        {
        case MetabolicEnergy:
            errorScore = (m_simulation->GetMetabolicEnergy() - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        case MechanicalEnergy:
            errorScore = (m_simulation->GetMechanicalEnergy() - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[indexNext], m_ValueList[indexNext], time));
            break;
        default:
            std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataType " << m_DataType << "\n";
        }
    }
    else
    {
        std::cerr << "DataTargetScalar::GetMatchValue error in " << m_Name << " unknown DataTarget " << m_DataType << "\n";
    }

    return errorScore;
}


void DataTargetScalar::Dump()
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
            *m_DumpStream << "Time\tTargetV\tActualV\tError\n";
        }
    }

    Body *body;
    Geom *geom;
    HingeJoint *hingeJoint;
    BallJoint *ballJoint;
    const double *r;
    double ref = 0;
    dVector3 result;
    dQuaternion q;
    TegotaeDriver *tegotaeDriver;

    if (m_DumpStream)
    {
        if ((body = dynamic_cast<Body *>(m_Target)) != 0)
        {
            switch (m_DataType)
            {
            case Q0:
                r = body->GetQuaternion();
                ref = r[0];
                break;
            case Q1:
                r = body->GetQuaternion();
                ref = r[1];
                break;
            case Q2:
                r = body->GetQuaternion();
                ref = r[2];
                break;
            case Q3:
                r = body->GetQuaternion();
                ref = r[3];
                break;
            case XP:
                r = body->GetPosition();
                ref = r[0];
                break;
            case YP:
                r = body->GetPosition();
                ref = r[1];
                break;
            case ZP:
                r = body->GetPosition();
                ref = r[2];
                break;
            case XV:
                r = body->GetLinearVelocity();
                ref = r[0];
                break;
            case YV:
                r = body->GetLinearVelocity();
                ref = r[1];
                break;
            case ZV:
                r = body->GetLinearVelocity();
                ref = r[2];
                break;
            case XRV:
                r = body->GetAngularVelocity();
                ref = r[0];
                break;
            case YRV:
                r = body->GetAngularVelocity();
                ref = r[1];
                break;
            case ZRV:
                r = body->GetAngularVelocity();
                ref = r[2];
                break;
            default:
                break;
            }
        }
        else if ((hingeJoint = dynamic_cast<HingeJoint *>(m_Target)) != 0)
        {
            hingeJoint->GetHingeAnchor(result);
            switch (m_DataType)
            {
            case XP:
                ref = result[0];
                break;
            case YP:
                ref = result[1];
                break;
            case ZP:
                ref = result[2];
                break;
            case Angle:
                ref = hingeJoint->GetHingeAngle();
            break;
            default:
                break;
            }
        }
        else if ((ballJoint = dynamic_cast<BallJoint *>(m_Target)) != 0)
        {
            ballJoint->GetBallAnchor(result);
            switch (m_DataType)
            {
            case XP:
                ref = result[0];
                break;
            case YP:
                ref = result[1];
                break;
            case ZP:
                ref = result[2];
                break;
            default:
                break;
            }
        }
        else if ((geom = dynamic_cast<Geom *>(m_Target)) != 0)
        {
            switch (m_DataType)
            {
            case Q0:
                geom->GetWorldQuaternion(q);
                ref = q[0];
                break;
            case Q1:
                geom->GetWorldQuaternion(q);
                ref = q[1];
                break;
            case Q2:
                geom->GetWorldQuaternion(q);
                ref = q[2];
                break;
            case Q3:
                geom->GetWorldQuaternion(q);
                ref = q[3];
                break;
            case XP:
                geom->GetWorldPosition(result);
                ref = result[0];
                break;
            case YP:
                geom->GetWorldPosition(result);
                ref = result[1];
                break;
            case ZP:
                geom->GetWorldPosition(result);
                ref = result[2];
                break;
            default:
                break;
            }
        }
        else if ((tegotaeDriver = dynamic_cast<TegotaeDriver *>(m_Target)) != 0)
        {
            switch (m_DataType)
            {
            case DriverError:
                double X = tegotaeDriver->X();
                double Y = tegotaeDriver->Y();
                ref = sqrt(X*X + Y*Y);
            }
        }
        else if (m_Target == 0)
        {
            switch(m_DataType)
            {
            case MetabolicEnergy:
                ref = m_simulation->GetMetabolicEnergy();
                break;
            case MechanicalEnergy:
                ref = m_simulation->GetMechanicalEnergy();
                break;
            default:
                break;
            }
        }

        int index = Util::BinarySearchRange(m_TargetTimeList, m_TargetTimeListLength, m_simulation->GetTime());
        *m_DumpStream << m_simulation->GetTime() <<
                "\t" << Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[index + 1], m_ValueList[index + 1], m_simulation->GetTime()) <<
                "\t" << ref <<
                "\t" << ref - Util::Interpolate(m_TargetTimeList[index], m_ValueList[index], m_TargetTimeList[index + 1], m_ValueList[index + 1], m_simulation->GetTime()) <<
                "\n";
    }
}

