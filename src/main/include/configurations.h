// configurations.h

#pragma once
#include "CAN_ID_DEFFINITIONS.cpp"


// #define KILL_NEW_FEATURE
// #define KILL_SPECIFIC_NEW_FEATURE
// #define COMPILE_FOR_RELEASE
#define DEBUGGING_FEATURE
// #define FEATURE_UNDER_DEVELOPMENT

////////////////
/*DEBUG LEVELS*/
// #define DEBUG_CURRENT
// #define DEBUG_OTHER
#define DEBUG_TUNABLE
/*DEBUG LEVELS*/
////////////////



#define NEW_FEATURES true


#ifdef KILL_NEW_FEATURE
    #define COMPILE_FOR_RELEASE
    #define NEW_FEATURES false
#endif

#ifdef COMPILE_FOR_RELEASE
    #undef DEBUGGING_FEATURE
    #undef FEATURE_UNDER_DEVELOPMENT
#endif

#if NEW_FEATURES
    #define MOTOR_CURRENT_LIMMITING
    #define MAX_SPEED_FIX
    // #define MAX_TEMP_COOLDOWN
    #define CAMERA_PAN_TILT
    #define CUSTOM_DRIVETRAIN
#else
    #undef MOTOR_CURRENT_LIMMITING
    #undef MAX_SPEED_FIX
    #undef MAX_TEMP_COOLDOWN
    #undef CAMERA_PAN_TILT
    #undef CUSTOM_DRIVETRAIN
#endif

#ifndef DEBUGGING_FEATURE
    #undef DEBUG_CURRENT
    #undef DEBUG_OTHER
    #undef DEBUG_TUNABLE
#endif

#ifdef KILL_SPECIFIC_NEW_FEATURE
    #undef CUSTOM_DRIVETRAIN
#endif


/////////////////////////////////////////////////////////
/*CONFIGURATION SETTINGS FOR MOTOR SIDES AND DIRECTIONS*/

/**
 * Swaps wich side the program thinks is left and right set
 * of motors if you suspect an issue, put the robot in tank
 * drive and test by moving either the left or right joystick
 * on the f310. Uncomment this line if the wrong side moves.
 * IGNORE THE EXPECTED DIRECTION
*/

#define SWAP_LEFT_MOTORS_WITH_RIGHT_MOTORS




/*if robot drives backward when forward is desired, uncomment this line*/

#define REVERSE_DEFFAULT_MOTOR_DIRECTION



/*CONFIGURATION SETTINGS FOR MOTOR SIDES AND DIRECTIONS*/
/////////////////////////////////////////////////////////

/**
 *
 *        layout_1        |       layout_2
 *                        |                 
 *                        |
 *       1  FRONT  4      |      1  FRONT  6
 *                        |
 *                        |
 *       2         5      |      2         5
 *                        |
 *                        |
 *       3  REAR   6      |      3  REAR   4
 *________________________|________________________
 *                        |
 *                        |
 *        layout_3        |       layout_4
 *                        |                 
 *                        |
 *       1  FRONT  4      |      1  FRONT  4
 *                        |
 *                        |
 *       3         6      |      3         6
 *                        |
 *                        |
 *       2  REAR   5      |      2  REAR   5
 *                        |
*/

// TODO: settup the left right forward backward code to match the configurations propperly
#define LAYOUT_1  // chose the layout from above that matches the robot
                  // and change the number after the underscore appropriatly

#ifdef LAYOUT_1
    #define LEFT_FRONT_MOTOR_CONTROLLER_CAN_ID     1
    #define LEFT_CENTER_MOTOR_CONTROLLER_CAN_ID    2
    #define LEFT_REAR_MOTOR_CONTROLLER_CAN_ID      3

    #define RIGHT_FRONT_MOTOR_CONTROLLER_CAN_ID    4
    #define RIGHT_CENTER_MOTOR_CONTROLLER_CAN_ID   5
    #define RIGHT_REAR_MOTOR_CONTROLLER_CAN_ID     6
#endif

#ifdef LAYOUT_2
    #define LEFT_FRONT_MOTOR_CONTROLLER_CAN_ID     1
    #define LEFT_CENTER_MOTOR_CONTROLLER_CAN_ID    2
    #define LEFT_REAR_MOTOR_CONTROLLER_CAN_ID      3

    #define RIGHT_FRONT_MOTOR_CONTROLLER_CAN_ID    6
    #define RIGHT_CENTER_MOTOR_CONTROLLER_CAN_ID   5
    #define RIGHT_REAR_MOTOR_CONTROLLER_CAN_ID     4
#endif

#ifdef LAYOUT_3
    #define LEFT_FRONT_MOTOR_CONTROLLER_CAN_ID     1
    #define LEFT_CENTER_MOTOR_CONTROLLER_CAN_ID    3
    #define LEFT_REAR_MOTOR_CONTROLLER_CAN_ID      2

    #define RIGHT_FRONT_MOTOR_CONTROLLER_CAN_ID    4
    #define RIGHT_CENTER_MOTOR_CONTROLLER_CAN_ID   6
    #define RIGHT_REAR_MOTOR_CONTROLLER_CAN_ID     5
#endif

#ifdef LAYOUT_4
    #define LEFT_FRONT_MOTOR_CONTROLLER_CAN_ID     2
    #define LEFT_CENTER_MOTOR_CONTROLLER_CAN_ID    1
    #define LEFT_REAR_MOTOR_CONTROLLER_CAN_ID      3

    #define RIGHT_FRONT_MOTOR_CONTROLLER_CAN_ID    5
    #define RIGHT_CENTER_MOTOR_CONTROLLER_CAN_ID   4
    #define RIGHT_REAR_MOTOR_CONTROLLER_CAN_ID     6
#endif

// #ifndef SWAP_LEFT_MOTORS_WITH_RIGHT_MOTORS
//     #ifdef REVERSE_DEFFAULT_MOTOR_DIRECTION
//     #endif
// #endif

// #define MOTOR_AND_GROUP_DECLARATIONS rev::CANSparkMax LeftFront       {LEFT_FRONT_MOTOR_CONTROLLER_CAN_ID,    BRUSHED};rev::CANSparkMax LeftCenter      {LEFT_CENTER_MOTOR_CONTROLLER_CAN_ID,   BRUSHED};rev::CANSparkMax LeftRear        {LEFT_REAR_MOTOR_CONTROLLER_CAN_ID,     BRUSHED};rev::CANSparkMax RightFront      {RIGHT_FRONT_MOTOR_CONTROLLER_CAN_ID,   BRUSHED};rev::CANSparkMax RightCenter     {RIGHT_CENTER_MOTOR_CONTROLLER_CAN_ID,  BRUSHED};rev::CANSparkMax RightRear       {RIGHT_REAR_MOTOR_CONTROLLER_CAN_ID,    BRUSHED};frc::MotorControllerGroup LeftMotors    { LeftFront,    LeftCenter,   LeftRear  };frc::MotorControllerGroup RightMotors   {RightFront,   RightCenter,   RightRear };
//     #ifndef SWAP_LEFT_MOTORS_WITH_RIGHT_MOTORS
//         frc::DifferentialDrive drivetrain {LeftMotors, RightMotors};
//     #else
//         frc::DifferentialDrive drivetrain {RightMotors, LeftMotors};
//     #endif