// Auton routine, moved out of main to reduce clutter

#include "autoRoutine.hpp"
#include "util/util.hpp"
#include "systemmanager.hpp"

// Hack to run driveToPointAsync as blocking
#define DRIVE_TO_POINT(x, y) util::runAsBlocking([&] { auton.driveToPointAsync({x, y}); }, [&] { return auton.isSettled(); });
// Hack to run turnToAngleAsync as blocking
#define TURN_TO_ANGLE_DEG(a) util::runAsBlocking([&] { auton.turnToAngleAsync(d2r(a)); }, [&] { return auton.isSettled(); });

void autoRoutine::skillsAuton()
{
    /*
    Starting on the red side, on the right of the middle goal
    "Up" = forward
    */

    //Moves the preload ball to the "upper" location, Also moves the upper roller to extend the hood
    util::runAsync([&] { indexer.getUpperBall(); });

    /* Getting the first red */
    DRIVE_TO_POINT(-72 + GOAL_RADIUS_IN / 2 + CHASSIS_WIDTH / 2, 24)
    intake.moveVelocity(-200); //Expand intake
    TURN_TO_ANGLE_DEG(90)
    intake.moveVelocity(0); //Stops the manual intake movement from expansion
    util::runAsync([&] { indexer.getLowerBall(); });
    DRIVE_TO_POINT(-36, 24)

    //Moves futher from the goal to prepare entering it at 45 degrees
    DRIVE_TO_POINT(-30, 30)
    TURN_TO_ANGLE_DEG(135)

    /* Scoring Right Bottom Goal*/

    //The middle goal has a very small tolerance, so we need as much accuracy as possible
    //Enters the goal slowly to minimize vibration from hitting the goal
    auton.withSpeed(0.75);
    DRIVE_TO_POINT(-17, 17)
    auton.resetSettings(); //Resets the settings to default.
    //The robot has 2 red balls
    indexer.score();        //Score the upper one
    pros::delay(500);       //Wait for scoring
    indexer.getUpperBall(); //Moves the lower ball to the higher position
    indexer.getLowerBall(); //Gets a blue ball to the lower position to descore it
    pros::delay(250);       //Waits a bit for stability

    intake.moveVelocity(-10); //Moves intake slowly backward to make sure we get the second blue ball
    DRIVE_TO_POINT(-36, 46)
    TURN_TO_ANGLE_DEG(170)
    indexer.discardLowerBall();
    TURN_TO_ANGLE_DEG(-90)

    /* Getting the red ball in the middle */
    util::runAsync([&] { indexer.getLowerBall(); });

    DRIVE_TO_POINT(-73.8, 46.5)

    /* Getting the middle bottom goal */
    TURN_TO_ANGLE_DEG(180)
    DRIVE_TO_POINT(-72, 18)

    indexer.score();
    pros::delay(250);
    //Resettings the position of the robot based on the heading.
    //The robot might approach the goal from different angles, so we can't just reset to a single position+heading
    //We calculate the robot's position using the heading and the distance between the robot's tracking center to the center of the goal
    odometry.reset(
        {-72 + (GOAL_RADIUS_IN / 2 + 9.25) * -sin(odometry.getPos().angle),
         5.8 + (GOAL_RADIUS_IN / 2 + 9.25) * -cos(odometry.getPos().angle),
         r2d(odometry.getPos().angle)},
        false);
    pros::delay(250);
    indexer.score();
    pros::delay(500);

    //Gets the lower blue ball to descore it.
    indexer.getLowerBall();

    //Moves the intake forward when backing out so we don't take the red ball out
    intake.moveVelocity(200);
    DRIVE_TO_POINT(-84, 48)
    intake.moveVelocity(0);

    //Discard the blue ball in the corner we started
    TURN_TO_ANGLE_DEG(130)
    indexer.discardLowerBall();

    /* Getting the lower left red ball */
    TURN_TO_ANGLE_DEG(-132)
    util::runAsync([&] { indexer.getUpperBall(); });
    DRIVE_TO_POINT(-108, 24 - 2)

    //Moves further from the goal to prepare entering it at 45 degrees
    DRIVE_TO_POINT(-142 + 34, 34)
    TURN_TO_ANGLE_DEG(-135)

    //Enters the goal fullspeed as we dont need as much accuracy as before
    DRIVE_TO_POINT(-142 + 17 - 3, 16 - 4)
    indexer.score();        //Scores the red ball
    indexer.getLowerBall(); //Gets the blue ball to descore it
    pros::delay(250);

    //Heading based reset
    odometry.reset({-144 + 5.8 + (GOAL_RADIUS_IN / 2 + 9.25) * -sin(odometry.getPos().angle), 5.8 + (GOAL_RADIUS_IN / 2 + 9.25) * -cos(odometry.getPos().angle), r2d(odometry.getPos().angle)}, false);
    //Will be {-138, 16.34, -135} ideally
    pros::delay(250);

    intake.moveVelocity(-10);              //Moves the intake slowly backward to grab onto the second blue ball
    DRIVE_TO_POINT(-144 + 48 - 3, 72 - 24) //Backing out of the goal
    indexer.discardLowerBall();

    /* Getting the two balls on the left-middle of the field */
    TURN_TO_ANGLE_DEG(0)
    util::runAsync([&] { indexer.getUpperBall(); });
    DRIVE_TO_POINT(-144 + 48 - 3, 72 - 2) //Driving to the first ball
    TURN_TO_ANGLE_DEG(-90)
    util::runAsync([&] { indexer.getLowerBall(); });

    //The second ball, the goal and the robot are all on the same line.
    //We split the movement to 2 parts to give the second ball time to go in
    DRIVE_TO_POINT(-144 + 26, 72 - 2)     //Driving to the second ball
    DRIVE_TO_POINT(-144 + 16 - 2, 72 - 2) //Driving to the goal

    /* The left-middle goal */
    indexer.score();
    pros::delay(250);
    // Heading based reset
    // We find that our haeding is off by 3 degrees here consistently, so we just added 3 degrees to it.
    // The skills run is coming to an end and we don't need the highest amount of accuracy
    odometry.reset({-144 + 5.8 + (GOAL_RADIUS_IN / 2 + 9.25) * -sin(odometry.getPos().angle), 72 + (GOAL_RADIUS_IN / 2 + 9.25) * -cos(odometry.getPos().angle), r2d(odometry.getPos().angle) + 3}, false);
    pros::delay(250);

    //Backing out of the goal and moving the remaining red ball to the upper position
    intake.moveVelocity(200);
    DRIVE_TO_POINT(-144 + 36, 73)
    intake.moveVelocity(0);
    util::runAsync([&] { indexer.getUpperBall(); });

    /* Left Top Goal */
    //Moves the intake outward incase we accidentally come into contact with the left top ball
    //This helps us in pushing the ball out of the way 
    intake.moveVelocity(50);
    TURN_TO_ANGLE_DEG(-16)
    DRIVE_TO_POINT(-144 + 24, 144 - 24)
    TURN_TO_ANGLE_DEG(-45)
    intake.moveVelocity(0);
    DRIVE_TO_POINT(-144 + 16 - 0.5, 144 - 16 - 2)//Last goal of the skills run, driving into it full speed

    indexer.score();
    pros::delay(250);
    //Heading based reset incase we add more things after this point
    odometry.reset({-142 + 5.8 + (GOAL_RADIUS_IN / 2 + 9.25) * -sin(odometry.getPos().angle), 142 - 5.8 + (GOAL_RADIUS_IN / 2 + 9.25) * -cos(odometry.getPos().angle), r2d(odometry.getPos().angle)}, false);
    pros::delay(250);

    //Spins intake backward to not pick up the blue ball, incase we want to rush the last few seconds to get another goal
    intake.moveVelocity(150);
    DRIVE_TO_POINT(-144+36, 144-46)
    intake.moveVelocity(0);
    TURN_TO_ANGLE_DEG(90)

    /* Getting the red ball in the middle */
    util::runAsync([&] { indexer.getUpperBall(); });

    DRIVE_TO_POINT(-73.8, 144-46);
    
    TURN_TO_ANGLE_DEG(0)
    DRIVE_TO_POINT(-72, 144-18)

    indexer.score();
    pros::delay(250);
    //Resettings the position of the robot based on the heading.
    //The robot might approach the goal from different angles, so we can't just reset to a single position+heading
    //We calculate the robot's position using the heading and the distance between the robot's tracking center to the center of the goal
    odometry.reset(
        {-72 + (GOAL_RADIUS_IN / 2 + 9.25) * -sin(odometry.getPos().angle),
         144-5.8 + (GOAL_RADIUS_IN / 2 + 9.25) * -cos(odometry.getPos().angle),
         r2d(odometry.getPos().angle)},
        false);
    pros::delay(250);

    //Gets the lower blue ball to descore it.
    indexer.getLowerBall();

    //Moves the intake forward when backing out so we don't take the red ball out
    intake.moveVelocity(200);
    DRIVE_TO_POINT(-60, 144-48)
    intake.moveVelocity(0);

    //Discard the blue ball in the corner we started
    TURN_TO_ANGLE_DEG(-40)
    indexer.discardLowerBall();

    /* Getting the lower left red ball */
    TURN_TO_ANGLE_DEG(42)
    util::runAsync([&] { indexer.getUpperBall(); });
    DRIVE_TO_POINT(-36, 144-22)

    //Moves further from the goal to prepare entering it at 45 degrees
    DRIVE_TO_POINT(-34, 144-34)
    TURN_TO_ANGLE_DEG(45)

    //Enters the goal fullspeed as we dont need as much accuracy as before
    DRIVE_TO_POINT(-17 + 3, 144 - (16 - 3))
    indexer.score();        //Scores the red ball
    pros::delay(250);

    //Heading based reset
    odometry.reset({5.8 + (GOAL_RADIUS_IN / 2 + 9.25) * -sin(odometry.getPos().angle), 144 - 5.8 + (GOAL_RADIUS_IN / 2 + 9.25) * -cos(odometry.getPos().angle), r2d(odometry.getPos().angle)}, false);
    pros::delay(250);

    intake.moveVelocity(150);
    DRIVE_TO_POINT(-48+3, 72 + 24) //Backing out of the goal
    TURN_TO_ANGLE_DEG(180)

    //End of our current skills autonomous routine
}
