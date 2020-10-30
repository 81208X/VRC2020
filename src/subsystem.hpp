#ifndef _SUBSYSTEM_HPP_INCLUDED
#define _SUBSYSTEM_HPP_INCLUDED

#include "io.hpp"
#include "subsystem/drive.hpp"
#include "subsystem/intake.hpp"
#include "subsystem/selfcheck.hpp"
#include "subsystem/heading.hpp"
#include "subsystem/vision.hpp"

extern DriveSubsystem drive;
extern IntakeSubsystem intake;
extern SelfCheck selfCheck;

extern HeadingSensor gyroSystem;

#ifdef VISION_SENSOR_LOWER
    extern VisionSensor visionSensorLower;
#endif

#ifdef VISION_SENSOR_UPPER
    extern VisionSensor visionSensorUpper;
#endif
#endif /* _SUBSYSTEM_HPP_INCLUDED */
