// Subsystems that interact with and abstract (or otherwise manage) other subsystems

#ifndef _SYSTEMMANAGER_HPP_INCLUDED
#define _SYSTEMMANAGER_HPP_INCLUDED

#include "subsystem.hpp"
#include "systemmanager/odometry.hpp"
#include "systemmanager/auto.hpp"
#include "systemmanager/indexer.hpp"


// Odometry
extern Odometry odometry;

// Auton
extern AutoDrive auton;

// Ball indexer
#if defined(UPPER_ROLLER_MOTOR) && defined(LOWER_ROLLER_MOTOR)
extern Indexer indexer;
#endif
#endif /* _SYSTEMMANAGER_HPP_INCLUDED */
