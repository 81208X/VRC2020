#include "vision.hpp"

VisionSensor::VisionSensor(uint8_t port, pros::vision_signature_s_t red, pros::vision_signature_s_t blue, uint8_t minWidth, uint8_t minHeight) : sensor(port) {
    // HACK: Hardcoded values from VCS utility, need to convert to PROS values and move to profiles
    // OLD VALUES
    // red = sensor.signature_from_utility(1, 9003, 10399, 9700, -1645, -527, -1086, 5.400, 0);
    // blue = sensor.signature_from_utility(2, -3745, -2765, -3256, 11467, 13825, 12646, 6.000, 0);
    // NEW VALUES
    red = sensor.signature_from_utility(1, 6509, 9317, 7913, -799, 1, -399, 3.100, 0);
    blue = sensor.signature_from_utility(2, -3325, -2367, -2846, 6061, 12483, 9272, 2.000, 0);
    sensor.set_signature(redSig, &red);
    sensor.set_signature(blueSig, &blue);
    minH = minHeight;
    minW = minWidth;
}

VisionSensor::VisionSensor(uint8_t port, uint8_t red, uint8_t blue, uint8_t minWidth, uint8_t minHeight) : sensor(port) {
    redSig = red;
    blueSig = blue;
    minH = minHeight;
    minW = minWidth;
}

bool VisionSensor::detectRedBall() {
    // Check at least one object is detected
    if(sensor.get_object_count() > 0) {
        obj = sensor.get_by_size(0); // Get largest object visible
        if(obj.width < minW || obj.height < minH)
            return false;
        return obj.signature == redSig;
    }
    else
        return false;
}

bool VisionSensor::detectBlueBall() {
    // Check at least one object is detected
    if(sensor.get_object_count() > 0) {
        obj = sensor.get_by_size(0); // Get largest object visible
        if(obj.width < minW || obj.height < minH)
            return false;
        return obj.signature == blueSig;
    }
    else
        return false;
}

bool VisionSensor::detectBall() {
    // Check at least one object is detected
    if(sensor.get_object_count() > 0) {
        obj = sensor.get_by_size(0); // Get largest object visible
        if(obj.width < minW || obj.height < minH)
            return false;
        // This check is actually redundant because red/blue sigs are the only
        // ones that should be available on the vision sensor when code is running.
        // If an object is detected, just return true.
        // return obj.signature == redSig || obj.signature == blueSig;
        return true;
    }
    else
        return false;
}
