/*
 *  NamedObject.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Root object that allows some basic data storage such as naming

#ifndef NamedObject_h
#define NamedObject_h

#include <string>
#include <iostream>
#include <sstream>

#ifdef USE_QT
#include "GLUtils.h"
#include <QtGlobal>
#include <QColor>
#endif

class FacetedObject;
class Simulation;

namespace rapidxml { template<class Ch> class xml_document; }
namespace rapidxml { template<class Ch> class xml_node; }
namespace rapidxml { template<class Ch> class xml_attribute; }

#ifdef USE_QT
namespace irr { namespace scene { class IMesh; } }
namespace irr { namespace video { class SColor; } }
class SimulationWindow;
#endif

class NamedObject
{
public:

    NamedObject();
    virtual ~NamedObject();

    void SetName(const char* name) { m_Name = name; }
    void SetName(const unsigned char* name) { m_Name = (const char *)name; }
    void SetName(const std::string &name) { m_Name = name; }
    void SetName(const std::string *name) { m_Name = *name; }
    const std::string *GetName() { return &m_Name; }

    bool GetVisible() { return m_Visible; }
    void SetVisible(bool v) { m_Visible = v; }

    bool GetDump() { return m_Dump; }
    void SetDump(bool v) { m_Dump = v; }

    bool GetCaseSensitiveXMLAttributes() { return m_CaseSensitiveXMLAttributes; }
    void SetCaseSensitiveXMLAttributes(bool v) { m_CaseSensitiveXMLAttributes = v; }

    void SetMessage(const char* message) { m_Message = message; }
    void SetMessage(const unsigned char* message) { m_Message = (const char *)message; }
    void SetMessage(const std::string &message) { m_Message = message; }
    void SetMessage(const std::string *message) { m_Message = *message; }
    const std::string *GetMessage() { return &m_Message; }

    virtual void Dump();
    virtual int XMLLoad(rapidxml::xml_node<char> * node);
    virtual rapidxml::xml_node<char> * XMLSave(rapidxml::xml_document<char> *doc);
    virtual void WriteToXMLStream(std::ostream &outputStream);

    Simulation *simulation() const;
    void setSimulation(Simulation *simulation);

#ifdef USE_QT
    void SetAxisSize(const float axisSize[3]) {m_AxisSize[0] = axisSize[0]; m_AxisSize[1] = axisSize[1]; m_AxisSize[2] = axisSize[2]; }
    void SetAxisSize(float axisSize) {m_AxisSize[0] = axisSize; m_AxisSize[1] = axisSize; m_AxisSize[2] = axisSize; }
    void SetColour(const Colour &colour) { m_Colour = colour; }
    void SetColour(const Colour *colour) { m_Colour = *colour; }
    void SetColour(const QColor &c) { m_Colour.SetColour(c.redF(), c.greenF(), c.blueF(), c.alphaF()); }
    void SetColour(float r, float g, float b, float alpha) { m_Colour.SetColour(r, g, b, alpha); }
    const float *GetAxisSize() { return m_AxisSize; }
    const Colour *GetColour() { return &m_Colour; }

    FacetedObject *physRep() const;
    void setPhysRep(FacetedObject *physRep);

    virtual void Draw(SimulationWindow *window) { Q_UNUSED(window); }
#endif

protected:

    void GetXMLAttribute(rapidxml::xml_node<char> *cur, const char *name, std::string *attributeValue);
    void GetXMLAttribute(rapidxml::xml_node<char> *cur, const char *name, char *attributeValue, int attributeValueLength);
    rapidxml::xml_attribute<char> *ReplaceXMLAttribute(rapidxml::xml_node<char> *cur, const char *name, const char *newValue);
    void RemoveXMLAttribute(rapidxml::xml_node<char> *cur, const char *name);
    rapidxml::xml_attribute<char> *FindXMLAttribute(rapidxml::xml_node<char> *cur, const char *name);

    std::string m_Name;
    std::string m_Message;

    bool m_Visible;

    bool m_Dump;
    bool m_FirstDump;
    std::ofstream *m_DumpStream;

    bool m_CaseSensitiveXMLAttributes;
    Simulation *m_simulation;

#ifdef USE_QT
    float m_AxisSize[3];
    Colour m_Colour;
    FacetedObject *m_physRep;
    bool m_FirstDraw;
#endif
};

#endif
