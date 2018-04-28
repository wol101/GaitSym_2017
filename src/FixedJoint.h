/*
 *  FixedJoint.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 20/09/2008.
 *  Copyright 2008 Bill Sellers. All rights reserved.
 *
 */

#ifndef FixedJoint_h
#define FixedJoint_h

#include "Joint.h"
#include "PGDMath.h"

#include <ode/ode.h>

#ifdef USE_QT
namespace irr { namespace video { class ITexture; } }
namespace irr { namespace video { class SMaterial; } }
namespace irr { namespace scene { class IMesh; } }
namespace irr { namespace scene { class ISceneNode; } }
#endif

class ButterworthFilter;
class MovingAverage;
class Filter;

class FixedJoint: public Joint
{
    public:

    FixedJoint(dWorldID worldID);
    ~FixedJoint();

    void SetFixed();

    void SetCrossSection(unsigned char *stiffness, int nx, int ny, double dx, double dy);
    void SetStressOrigin(double x, double y, double z);
    void SetStressOrigin(const char *buf);
    void SetStressOrientation(double q0, double q1, double q2, double q3);
    void SetStressOrientation(const char *buf);

    enum StressCalculationType { none = 0, beam, spring };
    void SetStressCalculationType(StressCalculationType type) { m_stressCalculationType = type; }

    pgd::Vector GetStressOrigin() { return m_StressOrigin; }
    pgd::Quaternion GetStressOrientation() { return m_StressOrientation; }

    double GetMaxStress() { return m_maxStress; }
    double GetMinStress() { return m_minStress; }

    void SetStressLimit(double stressLimit) { m_stressLimit = stressLimit; };
    bool CheckStressAbort();

    enum LowPassType { NoLowPass = 0, MovingAverageLowPass, Butterworth2ndOrderLowPass };
    void SetLowPassType(LowPassType lowPassType) { m_lowPassType = lowPassType; }
    LowPassType GetLowPassType() { return m_lowPassType; }
    void SetWindow(int window);
    void SetCutoffFrequency(double cutoffFrequency);
    double GetLowPassMinStress() { return m_lowPassMinStress; }
    double GetLowPassMaxStress() { return m_lowPassMaxStress; }

    double *GetStress() { return m_stress; }

    virtual void Update();
    virtual void Dump();

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window);
    void SetDisplayRange(double low, double high) { m_lowRange = low; m_highRange = high; }
#endif


    double maxStress() const;

protected:

    void CalculateStress();

    // these are used for the stress/strain calculations
    unsigned char *m_stiffness;
    double *m_stress;
    double *m_xDistances;
    double *m_yDistances;
    int m_nx;
    int m_ny;
    int m_nActivePixels;
    double m_dx;
    double m_dy;
    double m_Ix;
    double m_Iy;
    double m_Ixy;
    double m_area;
    double m_xOrigin;
    double m_yOrigin;
    double m_minStress;
    double m_maxStress;
    StressCalculationType m_stressCalculationType;

    pgd::Vector m_StressOrigin;
    pgd::Quaternion m_StressOrientation;
    pgd::Vector m_torqueStressCoords;
    pgd::Vector m_forceStressCoords;
    pgd::Vector m_torqueAxis;
    double m_torqueScalar;

    double m_stressLimit;
    Filter **m_filteredStress;
    double m_lowPassMinStress;
    double m_lowPassMaxStress;
    LowPassType m_lowPassType;

    pgd::Vector *m_vectorList;

    int m_dumpCount;

#ifdef USE_QT
    irr::scene::IMesh *createRectMesh(float xmin, float ymin, float xmax, float ymax, irr::video::SMaterial *material);

    double m_LastDisplayTime;
    irr::video::ITexture *m_textureImage;
    irr::scene::ISceneNode* m_displayRect;

    double m_lowRange;
    double m_highRange;
    unsigned char *m_colourMap;
#endif

};



#endif
