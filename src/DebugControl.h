/*
 *  DebugControl.h
 *  GaitSymODE
 *
 *  Created by Bill Sellers on 24/08/2005.
 *  Copyright 2005 Bill Sellers. All rights reserved.
 *
 */

// DebugControl.h - holds some values useful for debugging

#ifndef DEBUG_H
#define DEBUG_H

#include <iostream>

enum DebugControl
{
    NoDebug = 0,
    FitnessDebug,
    MainDebug,
    MAMuscleDebug,
    MAMuscleExtendedDebug,
    UGMMuscleDebug,
    DampedSpringDebug,
    MuscleDebug,
    MemoryDebug,
    SocketDebug,
    SimulationDebug,
    CylinderWrapStrapDebug,
    TwoCylinderWrapStrapDebug,
    JointDebug,
    ContactDebug,
    FacetedObjectDebug,
    EnergyPartitionDebug,
    CentreOfMassDebug,
    XMLDebug,
    MAMuscleCompleteDebug,
    ActivationSegmentStateDebug,
    UDPDebug,
    TCPDebug,
    ModelStateDebug,
    HingeJointDebug,
    StrapDebug,
    MAMuscleExtendedDampedDebug,
    ParserDebug,
    GLUIDebug
};

const char * const gDebugLabels[] =
{
    "NoDebug",
    "FitnessDebug",
    "MainDebug",
    "MAMuscleDebug",
    "MAMuscleExtendedDebug",
    "UGMMuscleDebug",
    "DampedSpringDebug",
    "MuscleDebug",
    "MemoryDebug",
    "SocketDebug",
    "SimulationDebug",
    "CylinderWrapStrapDebug",
    "TwoCylinderWrapStrapDebug",
    "JointDebug",
    "ContactDebug",
    "FacetedObjectDebug",
    "EnergyPartitionDebug",
    "CentreOfMassDebug",
    "XMLDebug",
    "MAMuscleCompleteDebug",
    "ActivationSegmentStateDebug",
    "UDPDebug",
    "TCPDebug",
    "ModelStateDebug",
    "HingeJointDebug",
    "StrapDebug",
    "MAMuscleExtendedDampedDebug",
    "ParserDebug",
    "GLUIDebug"
};

#ifndef DEBUG_MAIN
// cope with declaring globals extern
extern DebugControl gDebug;
extern std::ostream *gDebugStream;
extern std::string gDebugFunctionFilter;
extern std::string gDebugNameFilter;
#else
DebugControl gDebug = NoDebug;
std::ostream *gDebugStream = &std::cerr;
std::string gDebugFunctionFilter;
std::string gDebugNameFilter;
#endif

// returns true if OK to process
inline bool DebugFunctionFilter(const std::string &str)
{
    if (gDebugFunctionFilter.size() == 0) return true;
    if (gDebugFunctionFilter == str) return true;
    return false;
}

inline bool DebugNameFilter(const std::string &str)
{
    if (gDebugNameFilter.size() == 0) return true;
    if (gDebugNameFilter == str) return true;
    return false;
}

inline bool DebugFilters(const std::string &function, const std::string &name)
{
    if (DebugFunctionFilter(function) && DebugNameFilter(name)) return true;
    return false;
}

#endif // DEBUG_H
