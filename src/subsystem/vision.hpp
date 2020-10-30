#ifndef _VISION_HPP_INCLUDED
#define _VISION_HPP_INCLUDED

#include "api.h"

class VisionSensor {
private:
    pros::vision_object_s_t obj;
    uint8_t minH, minW, 
            // Use first two signature slots when using code-supplied sigs
            redSig = 1, blueSig = 2;
            
public:
    pros::Vision sensor; // Direct access to vison sensor to bypass wrapper
    // For using program-supplied vision signatures
    VisionSensor(uint8_t port, pros::vision_signature_s_t redSig, pros::vision_signature_s_t blueSig, uint8_t minW = 0, uint8_t minH = 0);
    // For using vision signatures programmed into the sensor
    VisionSensor(uint8_t port, uint8_t redSig, uint8_t blueSig, uint8_t minW = 0, uint8_t minH = 0);
    /**
	 * Checks for the presence of a red ball.
     *
     * \returns
     *        True if a ball red is present.
	 */
    bool detectRedBall();

    /**
	 * Checks for the presence of a blue ball.
     *
     * \returns
     *        True if a blue ball is present.
	 */
    bool detectBlueBall();

    /**
	 * Checks for the presence of a ball.
     *
     * \returns
     *        True if a ball is present.
	 */
    bool detectBall();
};
#endif /* _VISION_HPP_INCLUDED */
