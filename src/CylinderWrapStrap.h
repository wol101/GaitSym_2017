/*
 *  CylinderWrapStrap.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 29/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

#ifndef CylinderWrapStrap_h
#define CylinderWrapStrap_h

#include "Strap.h"
#include "PGDMath.h"

class CylinderWrapStrap: public Strap
{
public:

    CylinderWrapStrap();
    virtual ~CylinderWrapStrap();

    void SetOrigin(Body *body, dVector3 point);
    void SetInsertion(Body *body, dVector3 point);
    void SetOrigin(Body *body, const char *buf);
    void SetInsertion(Body *body, const char *buf);

    void SetCylinderBody(Body *body);
    void SetCylinderRadius(double radius) { m_CylinderRadius = radius; };
    void SetCylinderPosition(double x, double y, double z);
    void SetCylinderQuaternion(double q0, double q1, double q2, double q3);
    void SetCylinderAxis(double x, double y, double z);
    void SetCylinderPosition(const char *buf);
    void SetCylinderQuaternion(const char *buf);
    void SetCylinderAxis(const char *buf);
    void SetNumWrapSegments(int num) { m_NumWrapSegments = num; };

    virtual void Calculate(double deltaT);

    void GetOrigin(Body **body, dVector3 pos) { *body = m_OriginBody; pos[0] = m_OriginPosition.x; pos[1] = m_OriginPosition.y; pos[2] = m_OriginPosition.z; };
    void GetInsertion(Body **body, dVector3 pos) { *body = m_InsertionBody; pos[0] = m_InsertionPosition.x; pos[1] = m_InsertionPosition.y; pos[2] = m_InsertionPosition.z; };
    void GetCylinder(Body **body, dVector3 pos, double *radius, dQuaternion q);

    virtual int SanityCheck(Strap *otherStrap, AxisType axis, const std::string &sanityCheckLeft, const std::string &sanityCheckRight);

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
    void SetCylinderColour(const Colour &colour)  { m_CylinderColour = colour; };
    void SetCylinderLength(float length)  { m_CylinderLength = length; };
#endif

protected:

    int CylinderWrap(pgd::Vector &origin, pgd::Vector &insertion, double radius, int nWrapSegments, double maxAngle,
                     pgd::Vector &originForce, pgd::Vector &insertionForce, pgd::Vector &cylinderForce, pgd::Vector &cylinderForcePosition,
                     double *pathLength, pgd::Vector *pathCoordinates, int *numPathCoordinates);

    Body *m_OriginBody;
    pgd::Vector m_OriginPosition;
    Body *m_InsertionBody;
    pgd::Vector m_InsertionPosition;

    Body *m_CylinderBody;
    pgd::Vector m_CylinderPosition;
    pgd::Quaternion m_CylinderQuaternion;
    double m_CylinderRadius;
    int m_NumWrapSegments;

    int m_WrapStatus;

    pgd::Vector *m_PathCoordinates;
    int m_NumPathCoordinates;

#ifdef USE_QT
    Colour m_CylinderColour;
    float m_CylinderLength;
#endif
};

#endif

