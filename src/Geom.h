/*
 *  Geom.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 28/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Wrapper class to hold ODE geom

#ifndef Geom_h
#define Geom_h

#include "NamedObject.h"

#include <ode/ode.h>

#include <vector>

class Contact;
class SimulationWindow;

class Geom: public NamedObject
{
public:

    Geom();
    virtual ~Geom();

    enum GeomLocation
    {
        environment,
        body
    };

    void SetBody(dBodyID body);
    dBodyID GetBody();
    dGeomID GetGeomID() { return m_GeomID; }

    // these functions set the geom position relative to its body
    void SetPosition (double x, double y, double z);
    void SetQuaternion(double q0, double q1, double q2, double q3);
    void SetPosition (const char *buf);
    void SetQuaternion(const char *buf);

    // return body local values
    const double *GetPosition();
    void GetQuaternion(dQuaternion q);
    // return world values
    void GetWorldPosition(dVector3 p);
    void GetWorldQuaternion(dQuaternion q);


    void SetGeomLocation(GeomLocation l) { m_GeomLocation = l; };
    GeomLocation GetGeomLocation() { return m_GeomLocation; };

    void SetContactSoftCFM(double cfm) { m_CFM = cfm; };
    double GetContactSoftCFM() { return m_CFM; };
    void SetContactSoftERP(double erp) { m_ERP = erp; };
    double GetContactSoftERP() { return m_ERP; };
    void SetContactMu(double mu) { m_Mu = mu; };
    double GetContactMu() { return m_Mu; };
    void SetContactBounce(double bounce) { m_Bounce = bounce; };
    double GetContactBounce() { return m_Bounce; };

    void SetSpringDamp(double springConstant, double dampingConstant, double integrationStep);
    void SetSpringERP(double springConstant, double ERP, double integrationStep);

    void SetAbort(bool abort) { m_Abort = abort; };
    bool GetAbort() { return m_Abort; };

    void AddContact(Contact *contact) { m_ContactList.push_back(contact); }
    std::vector<Contact *> *GetContactList() { return &m_ContactList; }
    void ClearContacts() { m_ContactList.clear(); }

    virtual void Dump();

#ifdef USE_QT
    virtual void Draw(SimulationWindow *window) = 0;
#endif

protected:

    dGeomID m_GeomID;

    GeomLocation m_GeomLocation;

    double m_CFM;
    double m_ERP;
    double m_Mu;
    double m_Bounce;

    bool m_Abort;

    std::vector<Contact *> m_ContactList;
};


#endif
