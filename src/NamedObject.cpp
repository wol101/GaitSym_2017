/*
 *  NamedObject.cpp
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 19/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// Root object that allows some basic data storage such as naming

#include "NamedObject.h"
#include "FacetedObject.h"
#include "Simulation.h"

#include "rapidxml.hpp"

#include <ode/ode.h>

#include <sstream>
#include <typeinfo>
#ifdef STRINGS_H_NEEDED
#include <strings.h>
#endif

/* Visual C does not define these functions */
#if defined(_MSC_VER)
#define strcasecmp _stricmp
#endif

NamedObject::NamedObject()
{
    m_Visible = true;
    m_Dump = false;
    m_FirstDump = true;
    m_DumpStream = 0;
    m_CaseSensitiveXMLAttributes = false;
    m_simulation = 0;

#ifdef USE_QT
    m_AxisSize[0] = m_AxisSize[1] = m_AxisSize[2] = 1;
    m_Colour.SetColour(1, 1, 1, 1);
    m_physRep = 0;
    m_FirstDraw = true;
#endif
}

NamedObject::~NamedObject()
{
    if (m_DumpStream)
    {
        m_DumpStream->close();
        delete(m_DumpStream);
    }
#ifdef USE_QT
    if (m_physRep) delete m_physRep;
#endif
}

void NamedObject::Dump()
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
            *m_DumpStream << "Name\tm_Visible\n";
        }
    }

    if (m_DumpStream)
    {
        *m_DumpStream << m_Name << "\t" << m_Visible << "\n";
    }
}

// this function initialises the data in the object based on the contents
// of a libxml2 node. It uses information from the simulation as required
// to satisfy dependencies
// it returns 0 on success and __LINE__ on failure
int NamedObject::XMLLoad(rapidxml::xml_node<char> *node)
{
    std::string ID;
    GetXMLAttribute(node, "ID", &ID);
    if (ID.length() == 0) goto ERROR_EXIT;
    SetName(ID);
    return 0;

ERROR_EXIT:
    std::ostringstream ss;
    ss << "Error loading XML node: name=" << node->name() << " ID=" << ID;
    SetMessage(ss.str());
    return __LINE__;
}

// this function copies the data in the object to a libxml2 node that
// it creates internally. Ownership of this new node is passed to the calling
// routine. It uses information in simulation to control the format used
// and to provide dependencies.
// it returns 0 on success and __LINE__ on failure
rapidxml::xml_node<char> * NamedObject::XMLSave(rapidxml::xml_document<char> *doc)
{
    rapidxml::xml_node<char> *node = doc->allocate_node(rapidxml::node_element, "OBJECT");
    rapidxml::xml_attribute<char> *attr = doc->allocate_attribute("ID", m_Name.c_str());
    node->append_attribute(attr);
    return node;
}

// returns the value of a named attribute
// using caller provided string
// returns "" if attribute is not found
void NamedObject::GetXMLAttribute(rapidxml::xml_node<char> *cur, const char *name, std::string *attributeValue)
{
    if (m_CaseSensitiveXMLAttributes)
    {
        for (rapidxml::xml_attribute<char> *attr = cur->first_attribute(); attr; attr = attr->next_attribute())
        {
            if (strcasecmp(name, attr->name()) == 0)
            {
                *attributeValue = attr->value();
                return;
            }
        }
    }
    else
    {
        for (rapidxml::xml_attribute<char> *attr = cur->first_attribute(); attr; attr = attr->next_attribute())
        {
            if (strcmp(name, attr->name()) == 0)
            {
                *attributeValue = attr->value();
                return;
            }
        }
    }
    attributeValue->clear();
    return;
}


// returns the value of a named attribute
// using a caller provided char * buffer
// returns a zero length string if attribute is not found
// attributeValueLength is the maximum permitted size of attributeValue including the terminating zero
void NamedObject::GetXMLAttribute(rapidxml::xml_node<char> *cur, const char *name, char *attributeValue, int attributeValueLength)
{
    if (m_CaseSensitiveXMLAttributes)
    {
        for (rapidxml::xml_attribute<char> *attr = cur->first_attribute(); attr; attr = attr->next_attribute())
        {
            if (strcasecmp(name, attr->name()) == 0)
            {
                int n = attributeValueLength - 1;
                strncpy(attributeValue, attr->value(), n);
                attributeValue[n] = 0;
                return;
            }
        }
    }
    else
    {
        for (rapidxml::xml_attribute<char> *attr = cur->first_attribute(); attr; attr = attr->next_attribute())
        {
            if (strcmp(name, attr->name()) == 0)
            {
                int n = attributeValueLength - 1;
                strncpy(attributeValue, attr->value(), n);
                attributeValue[n] = 0;
                return;
            }
        }
    }
    attributeValue[0] = 0;
}

// replaces the value of a named attribute if it exists
// if it doesn't exist then a new attribute is created
// returns a pointer to the attribute
rapidxml::xml_attribute<char> *NamedObject::ReplaceXMLAttribute(rapidxml::xml_node<char> *cur, const char *name, const char *newValue)
{
    rapidxml::xml_attribute<char> *ptr = 0;
    if (m_CaseSensitiveXMLAttributes)
    {
        for (rapidxml::xml_attribute<char> *attr = cur->first_attribute(); attr; attr = attr->next_attribute())
        {
            if (strcasecmp(name, attr->name()) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }
    else
    {
        for (rapidxml::xml_attribute<char> *attr = cur->first_attribute(); attr; attr = attr->next_attribute())
        {
            if (strcmp(name, attr->name()) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }

    if (ptr) cur->remove_attribute(ptr);
    char *allocatedName = cur->document()->allocate_string(name);
    char *allocatedValue = cur->document()->allocate_string(newValue);
    ptr = cur->document()->allocate_attribute(allocatedName, allocatedValue);
    cur->append_attribute(ptr);
    return ptr;
}

// removes a named attribute if it exists
void NamedObject::RemoveXMLAttribute(rapidxml::xml_node<char> *cur, const char *name)
{
    rapidxml::xml_attribute<char> *ptr = 0;
    if (m_CaseSensitiveXMLAttributes)
    {
        for (rapidxml::xml_attribute<char> *attr = cur->first_attribute(); attr; attr = attr->next_attribute())
        {
            if (strcasecmp(name, attr->name()) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }
    else
    {
        for (rapidxml::xml_attribute<char> *attr = cur->first_attribute(); attr; attr = attr->next_attribute())
        {
            if (strcmp(name, attr->name()) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }

    if (ptr) cur->remove_attribute(ptr);
}

// returns a pointer to an attribute if it exists
rapidxml::xml_attribute<char> *NamedObject::FindXMLAttribute(rapidxml::xml_node<char> *cur, const char *name)
{
    rapidxml::xml_attribute<char> *ptr = 0;
    if (m_CaseSensitiveXMLAttributes)
    {
        for (rapidxml::xml_attribute<char> *attr = cur->first_attribute(); attr; attr = attr->next_attribute())
        {
            if (strcasecmp(name, attr->name()) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }
    else
    {
        for (rapidxml::xml_attribute<char> *attr = cur->first_attribute(); attr; attr = attr->next_attribute())
        {
            if (strcmp(name, attr->name()) == 0)
            {
                ptr = attr;
                break;
            }
        }
    }

    return ptr;
}

#ifdef USE_QT
FacetedObject *NamedObject::physRep() const
{
    return m_physRep;
}

void NamedObject::setPhysRep(FacetedObject *physRep)
{
    if (m_physRep) delete m_physRep;
    m_physRep = physRep;
}

#endif

Simulation *NamedObject::simulation() const
{
    return m_simulation;
}

void NamedObject::setSimulation(Simulation *simulation)
{
    m_simulation = simulation;
}

#if defined(RAPIDXML_NO_EXCEPTIONS)
// this is the required rapidxml error handler when RAPIDXML_NO_EXCEPTIONS is used to disable exceptions
void rapidxml::parse_error_handler(const char *what, void *where)
{
    std::cout << "rapidxml::parse_error_handler Parse error (what) " << what << "\n";
    std::cout << "(where) " << where << "\n";
}
#endif

void NamedObject::WriteToXMLStream(std::ostream &outputStream)
{
    // just output the class name as the tag and the name
    outputStream << "<" << typeid(*this).name() << " Name=\"" << m_Name << "\"\n";
}

