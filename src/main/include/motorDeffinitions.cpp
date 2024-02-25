// #pragma once

// #include "includePublic.h"


// #define BRUSHED rev::CANSparkMaxLowLevel::MotorType::kBrushed

// class RoverDrivetrainMotors
// {
// private:
//     /* data */
// public:
//     RoverDrivetrainMotors(int leftFrontCanID, int leftCenterCanID, int leftRearCanID, int rightFrontCanID, int rightCenterCanID, int rightRearCanID);
//     ~RoverDrivetrainMotors();
// };

// RoverDrivetrainMotors::RoverDrivetrainMotors(int leftFrontCanID, int leftCenterCanID, int leftRearCanID, int rightFrontCanID, int rightCenterCanID, int rightRearCanID) {
//     rev::CANSparkMax LeftFront      {leftFrontCanID,   BRUSHED};
//     rev::CANSparkMax LeftCenter      {leftCenterCanID,  BRUSHED};
//     rev::CANSparkMax LeftRear      {leftRearCanID,    BRUSHED};

//     rev::CANSparkMax RightFront    {rightFrontCanID,   BRUSHED};
//     rev::CANSparkMax RightCenter    {rightCenterCanID,  BRUSHED};
//     rev::CANSparkMax RightRear    {rightRearCanID,    BRUSHED};

//     frc::MotorControllerGroup LeftMotors    { LeftFront,    LeftCenter,   LeftRear  };
//     frc::MotorControllerGroup RightMotors   {RightFront,   RightCenter,   RightRear };
// }

// RoverDrivetrainMotors::~RoverDrivetrainMotors()
// {
// }
