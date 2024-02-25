#pragma once
// #include "configureBuild.cpp"
#include <rev/CANSparkMax.h>
#include "CAN_ID_DEFFINITIONS.cpp"
#include "keyWordConversions.h"
#include <frc/motorcontrol/MotorControllerGroup.h>

/////////////////////////////////////////////////////////
/*!!!!!!!!!!!!!!!!DONT TOUCH THIS CODE!!!!!!!!!!!!*/

enum DrivetrainLayout {
    LAYOUT_1,
    LAYOUT_2,
    LAYOUT_3,
    LAYOUT_4,
};


class Drivetrain {
    public:
        Drivetrain();
        void setupDrivetrain(DrivetrainLayout layout);
        // Add more methods as needed
    private:
        // Add motor controllers and other members
};


//////////////////////////////////
  /*motor controller declaarations*/

  rev::CANSparkMax LeftFront       {LEFT_FRONT_MOTOR_CONTROLLER_CAN_ID,    BRUSHED};
  rev::CANSparkMax LeftCenter      {LEFT_CENTER_MOTOR_CONTROLLER_CAN_ID,   BRUSHED};
  rev::CANSparkMax LeftRear        {LEFT_REAR_MOTOR_CONTROLLER_CAN_ID,     BRUSHED};

  rev::CANSparkMax RightFront      {RIGHT_FRONT_MOTOR_CONTROLLER_CAN_ID,   BRUSHED};
  rev::CANSparkMax RightCenter     {RIGHT_CENTER_MOTOR_CONTROLLER_CAN_ID,  BRUSHED};
  rev::CANSparkMax RightRear       {RIGHT_REAR_MOTOR_CONTROLLER_CAN_ID,    BRUSHED};

//   rev::CANSparkMax testController  {SINGLE_CONTROLLER_TEST_CAN_ID,         BRUSHED};

  frc::MotorControllerGroup LeftMotors    { LeftFront,    LeftCenter,   LeftRear  };
  frc::MotorControllerGroup RightMotors   {RightFront,   RightCenter,   RightRear };























/* 
#ifdef SWAP_LEFT_MOTORS_WITH_RIGHT_MOTORS
    #ifdef REVERSE_DEFFAULT_MOTOR_DIRECTION
        #undef REVERSE_DEFFAULT_MOTOR_DIRECTION
    #else
        #define REVERSE_DEFFAULT_MOTOR_DIRECTION
    #endif
#endif

#ifndef REVERSE_DEFFAULT_MOTOR_DIRECTION
    #define true
#else
    #define true
#endif
#define LAYOUT_1

#ifndef LAYOUT_1 || LAYOUT_2 || LAYOUT_3 || LAYOUT_4
    #error "Please choose a drivetrain layout in the configurations.h"
#endif
#ifndef !LAYOUT_1 || !LAYOUT_2 || !LAYOUT_3 || !LAYOUT_4
    #error "Please choose a drivetrain layout in the configurations.h"
#endif

#ifdef LAYOUT_1
    // #include "robot_h_files/Robot_1.h"
    #define "Robot.h"
#endif

#ifdef LAYOUT_2
    #include "robot_h_files/Robot_2.h"
#endif

#ifdef LAYOUT_3
    #include "robot_h_files/Robot_3.h"
#endif

#ifdef LAYOUT_4
    #include "robot_h_files/Robot_4.h"
#endif
 */
/*!!!!!!!!!!!!!!!!DONT TOUCH THIS CODE!!!!!!!!!!!!*/
/////////////////////////////////////////////////////////