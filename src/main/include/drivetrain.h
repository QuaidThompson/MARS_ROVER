// settup motor controllers for the drivetrain

#include "CAN_ID_DEFFINITIONS.cpp"
#include "keyWordConversions.h"

#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>



/////////////////////////////////////////////////////////
/*CONFIGURATION SETTINGS FOR MOTOR SIDES AND DIRECTIONS*/

/*swaps wich side the program thinks is left and right set of motors
if you suspect an issue, put the robot in tank drive and test by moving
either the left or right joystick on the f310. Uncomment this line if the wrong side moves.
IGNORE THE EXPECTED DIRECTION*/

// #define SWAP_LEFT_MOTORS_WITH_RIGHT_MOTORS




/*if robot drives backward when forward is desired, uncomment this line*/

// #define REVERSE_DEFFAULT_MOTOR_DIRECTION



/*CONFIGURATION SETTINGS FOR MOTOR SIDES AND DIRECTIONS*/
/////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////
/*!!!!!!!!!!!!!!!!DONT TOUCH THIS CODE!!!!!!!!!!!!*/

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

/*!!!!!!!!!!!!!!!!DONT TOUCH THIS CODE!!!!!!!!!!!!*/
/////////////////////////////////////////////////////////





rev::CANSparkMax LeftFront       {LEFT_FRONT_MOTOR_CONTROLLER_CAN_ID,    BRUSHED};
rev::CANSparkMax LeftCenter      {LEFT_CENTER_MOTOR_CONTROLLER_CAN_ID,   BRUSHED};
rev::CANSparkMax LeftRear        {LEFT_REAR_MOTOR_CONTROLLER_CAN_ID,     BRUSHED};

rev::CANSparkMax RightFront      {RIGHT_FRONT_MOTOR_CONTROLLER_CAN_ID,   BRUSHED};
rev::CANSparkMax RightCenter     {RIGHT_CENTER_MOTOR_CONTROLLER_CAN_ID,  BRUSHED};
rev::CANSparkMax RightRear       {RIGHT_REAR_MOTOR_CONTROLLER_CAN_ID,    BRUSHED};

frc::MotorControllerGroup LeftMotors    { LeftFront,    LeftCenter,   LeftRear  };
frc::MotorControllerGroup RightMotors   {RightFront,   RightCenter,   RightRear };


#ifndef SWAP_LEFT_MOTORS_WITH_RIGHT_MOTORS
    frc::DifferentialDrive drivetrain {LeftMotors, RightMotors};
#else
    frc::DifferentialDrive drivetrain {RightMotors, LeftMotors};
#endif