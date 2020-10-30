#ifndef _PROFILES_HPP_INCLUDED
#define _PROFILES_HPP_INCLUDED

// Common bot options
#define LOGO_NAME leroi
#define CONFIGFILE "/usd/configs.txt"
#define LOGFILE "/usd/log.txt"
#define DEBUG_PORT "/dev/20" // putting down port 20 because its *generally* not used

/* Other field measurements */
#define GOAL_RADIUS_IN 11.29

/* Wheel related */
#define WHEEL_DIAM_REAL_IN 4.02

/** 
 * HACK: Our motors are 600 rpm, geared 7:3 to 257rpm, but we are already so used to having "200" being the max speed.
 * 
 * Solution: increasing the wheel diameter to "simulate" 200rpm
 * A 5.1657 inch wheel running at 200rpm moves at the same speed as a 4.02 inch wheel at 257 rpm.
 * 257 rpm * 4.02 inch diameter = 200 rpm * 5.1657 inch diameter = 3245 inches per minute
 */
#define WHEEL_DIAM_IN (WHEEL_DIAM_REAL_IN / 200.0 * (600.0 / 7.0 * 3.0)) // 5.167 inch approx

// Tracking wheel diameters
#define SIDE_DIAM_IN 2.732
#define BACK_DIAM_IN 3.285

/* Converting encoder ticks to inches */
#define TICKS_PER_ROTATION 360
#define SIDE_ENC_TO_IN SIDE_DIAM_IN * M_PI / 360
#define BACK_ENC_TO_IN BACK_DIAM_IN * M_PI / 360

/* Chassis measurements */
#define CHASSIS_WIDTH 13.5
#define BASE_WIDTH_IN 11 //13.75 - 1.38 * 2
#define BASE_LENGTH_IN 9
#define ENC_BASE_WIDTH_IN 9.2 // DOUBT, probably not accurate. TODO: re-measure
#define BACK_TO_CENTER_IN 2.12 // Distance between back encoder and tracking center
#define STARTING_Y_IN 8 // Distance between tracking center and wall when robot is placed back to wall TODO: measure this

// Distance between tracking center to a wheel
#define WHEEL_TO_CENTER_IN 7.10633 //sqrt((chassiswidth/2)^2 + (chassislength/2)^2)

/* Inverse kinematics related */
// Max speed of wheels in inches per second
#define MAX_SPEED_IN_S (WHEEL_DIAM_IN * M_PI * 200 / 60.0) // around 54
// Max rotation speed of the chassis, in radians per second
#define MAX_CHASSIS_RPS (MAX_SPEED_IN_S / WHEEL_TO_CENTER_IN) // around 7.6, which is around 1.2 rotations per second



/* Automatic Driving Values */
//PID values
#define FORWARD_P 0.045
#define FORWARD_I 0.0015
#define FORWARD_D -0.00275
#define STRAFE_P 0.055
#define STRAFE_I 0
#define STRAFE_D 0
#define TURNING_P 0.6
#define TURNING_I 0
#define TURNING_D 0.0013
//Stalling values
#define DRIVE_STALL_TORQUE 0.9
#define DRIVE_STALL_VELOCITY 10

// Slewrate limits. 
//
// The power forward, strafe and turn speeds are not allowed to change at a rate:
// higher than [acceleration value]/1s when increasing
// or
// higher than [deceleration value]/1s when decreasing
#define FORWARD_ACCEL 5
#define FORWARD_DECEL 7
#define STRAFE_ACCEL 5
#define STRAFE_DECEL 7
#define TURNING_ACCEL 5
#define TURNING_DECEL 7

/* Controller Mappings */
#define DEBUG_STRAIGHT      pros::E_CONTROLLER_DIGITAL_X
#define DEBUG_SORTER_B      pros::E_CONTROLLER_DIGITAL_B
#define DEBUG_SORTER_Y      pros::E_CONTROLLER_DIGITAL_Y
#define DEBUG_SORTER_DOWN   pros::E_CONTROLLER_DIGITAL_DOWN

#define BTN_EXPAND          pros::E_CONTROLLER_DIGITAL_LEFT

#define INTAKE_OUT          pros::E_CONTROLLER_DIGITAL_L1
#define INTAKE_IN           pros::E_CONTROLLER_DIGITAL_L2

#define ROLLER_OUT          pros::E_CONTROLLER_DIGITAL_R1
#define ROLLER_IN           pros::E_CONTROLLER_DIGITAL_R2

#define DRIVE_POWER         pros::E_CONTROLLER_ANALOG_LEFT_Y
#define DRIVE_STRAFE        pros::E_CONTROLLER_ANALOG_LEFT_X
#define DRIVE_TURN          pros::E_CONTROLLER_ANALOG_RIGHT_X

/* Ports and Motors */
// The temperature at which an overheating warning will be triggered
#define MOTOR_OVERHEAT_TEMP           55

// Drive motors
#define LEFT_FORWARD_MOTOR            1
#define LEFT_FORWARD_MOTOR_REVERSED   true
#define LEFT_REAR_MOTOR               4
#define LEFT_REAR_MOTOR_REVERSED      true
#define RIGHT_FORWARD_MOTOR           2
#define RIGHT_FORWARD_MOTOR_REVERSED  false
#define RIGHT_REAR_MOTOR              3
#define RIGHT_REAR_MOTOR_REVERSED     false

#define LEFT_INTAKE_MOTOR             7
#define LEFT_INTAKE_MOTOR_REVERSED    true
#define RIGHT_INTAKE_MOTOR            8
#define RIGHT_INTAKE_MOTOR_REVERSED   false

#define UPPER_ROLLER_MOTOR            10
#define UPPER_ROLLER_MOTOR_REVERSED   false
#define LOWER_ROLLER_MOTOR            9
#define LOWER_ROLLER_MOTOR_REVERSED   true

#define INERTIAL_SENSOR               5

// Vision/indexer
#define VISION_SENSOR_LOWER           6
#define VISION_LOWER_RED_SIG          {1, {1, 0, 0}, 1.500000, 4481, 8513, 6498, -641, 1407, 382, 3941159, 0}
#define VISION_LOWER_BLUE_SIG         {2, {1, 0, 0}, 4.400000, -3733, -2431, -3082, 11609, 15549, 13580, 1055565, 0}
#define VISION_LOWER_SIG_MIN_WIDTH    20
#define VISION_LOWER_SIG_MIN_HEIGHT   20

#define VISION_LOWER_INTAKE_MIN_X_POS 200
#define VISION_LOWER_INTAKE_MIN_Y_POS 90

#define INDEXER_FRONT                       'H'
#define INDEXER_FRONT_DETECTION_THRESHOLD   2300
#define INDEXER_BACK                        'G'
#define INDEXER_BACK_DETECTION_THRESHOLD    1650
#define INDEXER_BACK_DETECTION_THRESHOLD2   2650

// Encoders
#define BACK_ENCODER_TOP              'C'
#define BACK_ENCODER_BOTTOM           'D'
#define BACK_ENCODER_REVERSED         true

#define LEFT_ENCODER
#define LEFT_ENCODER_TOP              'E'
#define LEFT_ENCODER_BOTTOM           'F'
#define LEFT_ENCODER_REVERSED         true

#define RIGHT_ENCODER
#define RIGHT_ENCODER_TOP             'A'
#define RIGHT_ENCODER_BOTTOM          'B'
#define RIGHT_ENCODER_REVERSED        false

// Getting that 3wire expander would be really cool wouldn't it...

// misc
#define SELFCHECK_PORTS {                       \
                        RIGHT_FORWARD_MOTOR,  \
                        LEFT_FORWARD_MOTOR,   \
                        RIGHT_REAR_MOTOR,     \
                        LEFT_REAR_MOTOR,      \
                        LEFT_INTAKE_MOTOR,    \
                        RIGHT_INTAKE_MOTOR,   \
                        UPPER_ROLLER_MOTOR,   \
                        LOWER_ROLLER_MOTOR    \
                        }

// Debug settings
#define GYRO_LCD_LINE 4
#define ODON_LCD_LINE 1
// #define ODON_LCD_LINE_2 6
#define AUTO_LCD_LINE 6
#endif /* _PROFILES_HPP_INCLUDED */
