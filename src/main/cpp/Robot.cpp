// Robot.cpp

#include "configurations.h"
#include "Robot.h"
#include <iostream>
#include <algorithm>
#include <cmath>



#define DEBUG
#define PRINT

#ifdef DEBUG
#define dbg(x) std::cout << "DEBUG::: " << (x);
#define dbgln(x) std::cout << "DEBUG::: " << (x) << "\n";
#define dbgv(x) std::cout << (x);
#define dbgvln(x) std::cout << (x) << "\n";
#else
#define dbg(x)
#define dbgln(x)
#define dbgv(x)
#define dbgvln(x)
#endif

#ifdef PRINT
#define print(x) std::cout << (x);
#define println(x) std::cout << (x) << "\n";
#endif
#ifndef PRINT
#define print(x)
#define println(x)
#endif

#define F310_LEFT_STICK_X_AXIS 0
#define F310_LEFT_STICK_Y_AXIS 1
#define F310_RIGHT_STICK_X_AXIS 4
#define F310_RIGHT_STICK_Y_AXIS 5
#define F310_A_BUTTON 1
#define F310_B_BUTTON 2
#define F310_X_BUTTON 3
#define F310_Y_BUTTON 4

#define LEONARDO_STEERING_JOYSTICK 0
#define LEONARDO_SPEED_JOYSTICK 1
#define LEONARDO_CAMERA_PAN 4
#define LEONARDO_CAMERA_TILT 5

#define LEONARDO_HEADLIGHT_SWICH 4    // SHOULD BE 'Y' BUTTON
#define LEONARDO_SURFACELIGHT_SWICH 1 // SHOULD BE 'A' BUTTON
#define LEONARDO_UPDATE_INFO_BUTTON 3 // SHOULD BE 'X'BUTTON

// TODO: MOTOR TUNING
#define MAX_FORWARD_SPEED .3        // TUNE FOR THE TOP SPEED FORWARD
#define MAX_BACKWARD_SPEED -1       // TUNE FOR THE TOP SPEED BACKWARD
#define MIN_FORWARD_SPEED_TO_MOVE 0 // TUNE TO GET RID OF MOTOR DEADZONE
#define MIN_FORWARD_SPEED_TO_MOVE 0 // TUNE TO GET RID OF MOTOR DEADZONE
#define TESTING_RAMP_INCRIMENT .01  // TUNE TO SET HOW FAST THE MOTORS RAMP UP TO SPEED
#define MAX_TILT_ANGLE 150
#define MIN_TILT_ANGLE 45
#define MAX_PAN_ANGLE 160
#define MIN_PAN_ANGLE 15
#define PAN_CENTER 90
#define TILT_CENTER 90

#define MAX_MOTOR_OUTPUT 0.3
#define MAX_ROTATION_OUTPUT 0.3

#ifdef MOTOR_CURRENT_LIMMITING
#define MAX_CURRENT 100
#define NUM_OF_MOTORS 6
#endif

#ifdef FEATURE_UNDER_DEVELOPMENT
  /*               PID Controls               */
  /*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

  // TODO: calculate pid correction

  /*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
  /*               PID Controls               */
  //////////////////////////
  /*Encoder PID Paramaters*/
double kP = 0.1, kI = 0.0001, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
// #define kP              0.1
// #define kI              0.0001
// #define kD              1
// #define kIz             0
// #define kFF             0
// #define kMaxOutput      1
// #define kMinOutput     -1
/*Encoder PID Paramaters*/
//////////////////////////
#endif

float motorSpeed = 0;
bool motorsDrivingForward = true;
// bool headlightOutput = false;
// bool surfaceLightOutput = false;
bool infoUpdateButtonState = false;
bool prevInfoUpdateButtonState = false;
bool requestData = false;
bool useF310ForControl = false;
int servoAngle = 90;
bool direction = false;
// double temperatureCelsius = PowerDistrobutionP.GetTemperature();
double maxCurrent = 0;
float newSpeed = 0;
float newRotation = 0;
float rampIncrementSpeed = .08;
float rampIncrementRotation = .08;
float maxRotation = 1; // .5
float maxSpeed = .9;   //  .3

float panAngle = PAN_CENTER;
float tiltAngle = TILT_CENTER;

float LeftTankSpeed;
float RightTankSpeed;

double smoothingVal = .09;

double  leftFrontParams[] = { 0, 0.3, 0, -0.3 };
double leftCenterParams[] = { 0, 0.5, 0, -0.5 };
double   leftRearParams[] = { 0, 0.3, 0, -0.3 };

double  rightFrontParams[] = { 0, 0.3, 0, -0.3 };
double rightCenterParams[] = { 0, 0.5, 0, -0.5 };
double   rightRearParams[] = { 0, 0.3, 0, -0.3 };

double LeftFrontMinForward = 0;
double LeftCenterMinForward = 0;
double LeftRearMinForward = 0;
double RightFrontMinForward = 0;
double RightCenterMinForward = 0;
double RightRearMinForward = 0;

double LeftFrontMaxForward = 1;
double LeftCenterMaxForward = 1;
double LeftRearMaxForward = 1;
double RightFrontMaxForward = 1;
double RightCenterMaxForward = 1;
double RightRearMaxForward = 1;

double LeftFrontMinBackward = 0;
double LeftCenterMinBackward = 0;
double LeftRearMinBackward = 0;
double RightFrontMinBackward = 0;
double RightCenterMinBackward = 0;
double RightRearMinBackward = 0;

double LeftFrontMaxBackward = -1;
double LeftCenterMaxBackward = -1;
double LeftRearMaxBackward = -1;
double RightFrontMaxBackward = -1;
double RightCenterMaxBackward = -1;
double RightRearMaxBackward = -1;

/*FUNCTIONS (need to be moved to a seperate file to keep this one clean)*/
/*vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv*/

float RampVal(float currentVal, float targetVal, float rampIncriment) {
  if (currentVal != targetVal) { // do we need to ramp?
    if (currentVal > targetVal) { // if we are ramping down
      float f = currentVal - targetVal;
      if (f > rampIncriment) {
        currentVal -= rampIncriment;
        return currentVal;
      }
      else {
        return targetVal;
      }
    }
    else if (currentVal < targetVal) { // if we are ramping up
      float f = targetVal - currentVal;
      if (f > rampIncriment) {
        currentVal += rampIncriment;
        return currentVal;
      }
      else {
        return targetVal;
      }
    }
    else {
      print("!!! N.F.G. !!!"); // shouldnt ever get here
      return 0;
    }
  }
  else { // if we are already at the intended value
    return targetVal;
  }
}

float map(float input, float inA, float inb, float outA, float outB) { // map function for easy conversion of input to output values
  float output = outA + ((outB - outA) / (inb - inA)) * (input - inA);
  return output;
}

void Robot::setDrivetrain(double rotate, double drive, double mixConstant) { // todo: handle false zero turn as well as mix Constant input.
  // cws4
  //  mixConstant: range is 0+ to 1.0.   mixConstant of a low value will prevent a great speed difference between left and right.
  //  This code doesn't allow for opposite spin of left and right side (to avoid robot digging into gravel and not spinning).
  //  This code allows for mixConstant to limit a great speed difference between left and right side (to avoid robot possibly digging into gravel).
  double p_leftSpeed, p_rightSpeed;
  double absFastSpeed, absLowSpeed;
  double fastSpeed, lowSpeed;
  double maxDifference = 1.0;
  int location = 0;

  absFastSpeed = std::abs(drive);
  absLowSpeed = absFastSpeed - map(std::abs(rotate), 0, 1., 0, absFastSpeed * mixConstant); // don't let the rotate/steering get larger than the drive/speed.

  if (drive >= 0) {
    fastSpeed = absFastSpeed;
    lowSpeed = absLowSpeed;
  }
  else // (drive < 0)
  {
    fastSpeed = -absFastSpeed;
    lowSpeed = -absLowSpeed;
  }

  if (rotate >= 0) {
    p_leftSpeed = fastSpeed;
    p_rightSpeed = lowSpeed;
  }
  else // (rotate < 0)
  {
    p_leftSpeed = lowSpeed;
    p_rightSpeed = fastSpeed;
  }

  if (drive > 0) {
    if (p_leftSpeed > p_rightSpeed) {
      location = 1;
      if ((p_leftSpeed - p_rightSpeed) > maxDifference) {
        p_rightSpeed = p_leftSpeed - maxDifference;
      }
    }
    else if (p_leftSpeed < p_rightSpeed) {
      location = 2;
      if ((p_rightSpeed - p_leftSpeed) > maxDifference) {
        p_leftSpeed = p_rightSpeed - maxDifference;
      }
    }
  }
  else if (drive < 0) {
    if (p_leftSpeed > p_rightSpeed) {
      location = 3;
      if ((p_leftSpeed - p_rightSpeed) > maxDifference) {
        p_leftSpeed = p_rightSpeed + maxDifference;
      }
    }
    else if (p_leftSpeed < p_rightSpeed) {
      location = 4;
      if ((p_rightSpeed - p_leftSpeed) > maxDifference) {
        p_rightSpeed = p_leftSpeed + maxDifference;
      }
    }
  }

  LeftFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  LeftCenter.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  LeftRear.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  RightFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  RightCenter.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  RightRear.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  LeftFront.Set(p_leftSpeed);
  LeftCenter.Set(p_leftSpeed);
  LeftRear.Set(p_leftSpeed);

  RightFront.Set(p_rightSpeed);
  RightCenter.Set(p_rightSpeed);
  RightRear.Set(p_rightSpeed);

  frc::SmartDashboard::PutNumber("p_leftSpeed", -p_leftSpeed);
  frc::SmartDashboard::PutNumber("p_rightSpeed", -p_rightSpeed);
  frc::SmartDashboard::PutNumber("set loc.", location);

} // SetDriveTrain()


void Robot::setDrivetrainNoLimmit(double rotate, double drive, double mixConstant) { // todo: handle false zero turn as well as mix Constant input.
  // cws4
  //  mixConstant: range is 0+ to 1.0.   mixConstant of a low value will prevent a great speed difference between left and right.
  //  This code doesn't allow for opposite spin of left and right side (to avoid robot digging into gravel and not spinning).
  //  This code allows for mixConstant to limit a great speed difference between left and right side (to avoid robot possibly digging into gravel).
  double p_leftSpeed, p_rightSpeed;
  double absFastSpeed, absLowSpeed;
  double fastSpeed, lowSpeed;
  double maxDifference = 1.0;
  int location = 0;

  absFastSpeed = std::abs(drive);
  absLowSpeed = absFastSpeed - map(std::abs(rotate), 0, 1., 0, absFastSpeed * mixConstant); // don't let the rotate/steering get larger than the drive/speed.

  if (drive >= 0) {
    fastSpeed = absFastSpeed;
    lowSpeed = absLowSpeed;
  }
  else // (drive < 0)
  {
    fastSpeed = -absFastSpeed;
    lowSpeed = -absLowSpeed;
  }

  if (rotate >= 0) {
    p_leftSpeed = fastSpeed;
    p_rightSpeed = lowSpeed;
  }
  else // (rotate < 0)
  {
    p_leftSpeed = lowSpeed;
    p_rightSpeed = fastSpeed;
  }

  LeftFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  LeftCenter.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  LeftRear.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  RightFront.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  RightCenter.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  RightRear.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  LeftFront.Set(p_leftSpeed);
  LeftCenter.Set(p_leftSpeed);
  LeftRear.Set(p_leftSpeed);

  RightFront.Set(p_rightSpeed);
  RightCenter.Set(p_rightSpeed);
  RightRear.Set(p_rightSpeed);

  frc::SmartDashboard::PutNumber("p_leftSpeed", -p_leftSpeed);
  frc::SmartDashboard::PutNumber("p_rightSpeed", -p_rightSpeed);
  frc::SmartDashboard::PutNumber("set loc.", location);

} // setDrivetrainNoLimmit()


double Robot::GetJoyWithDZ(double joystickVal, double minPosVal, double maxNegVal) {
  if (joystickVal >= minPosVal) {
    return map(joystickVal, minPosVal, 1, 0, 1);
  }
  else if (joystickVal <= maxNegVal) {
    return map(joystickVal, minPosVal, -1, 0, -1);
  }
  else {
    return 0;
  }
}


void SendData() {
  // TODO: set up shuffleboard variables
  // only on update
  // weather
  // x accel
  // y accel
  // z accel
  // camera pan angle
  // camera tilt angle

  // send real time variables
  // battery voltage
  // motor1 current
  // motor2 current
  // motor3 current
  // motor4 current
  // motor5 current
  // motor6 current
  // logic to determine wether or not to send real time variables
  if (requestData) {
    // send weather and other misc data
  }
}




/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/
/*FUNCTIONS (need to be moved to a seperate file to keep this one clean)*/

#ifndef DEBUG_TUNABLE
void Robot::RobotInit() {
// frc::SmartDashboard::PutBoolean("DEBUG_TUNABLE", false);
  #ifndef REVERSE_DEFFAULT_MOTOR_DIRECTION
  LeftMotors.SetInverted(true);
  RightMotors.SetInverted(false);
  #else
  LeftMotors.SetInverted(false);
  RightMotors.SetInverted(true);
  #endif

  #ifdef MOTOR_CURRENT_LIMMITING
  LeftFront.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);
  LeftCenter.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);
  LeftRear.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);
  RightFront.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);
  RightCenter.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);
  RightRear.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);
  #endif

  #ifdef FEATURE_UNDER_DEVELOPMENT
  LeftFrontPIDController.SetFeedbackDevice(LeftFrontEncoder);
  LeftFrontPIDController.SetP(kP);
  LeftFrontPIDController.SetI(kI);
  LeftFrontPIDController.SetD(kD);
  LeftFrontPIDController.SetIZone(kIz);
  LeftFrontPIDController.SetFF(kFF);
  LeftFrontPIDController.SetOutputRange(kMinOutput, kMaxOutput);
  #endif
    // TODO: set drivetrain configuration
    // TODO: set all drivetrain motors to break mode  probably need to create my own class that contains the drivetrain motors and has functions to update the different parameters like break mode*
  frc::CameraServer::StartAutomaticCapture();
  #ifdef FEATURE_UNDER_DEVELOPMENT
  frc::CameraServer::StartAutomaticCapture(1);
  #endif

  #ifdef DEBUG_CURRENT
    // #pragma message("frc::SmartDashboard::PutNumber("MAX Current", maxCurrent);")
    // frc::SmartDashboard::PutNumber("MAX Current", maxCurrent);
  #endif

  #ifdef FEATURE_UNDER_DEVELOPMENT
    // frc::SmartDashboard::PutNumber("P Gain", kP);
    // frc::SmartDashboard::PutNumber("I Gain", kI);
    // frc::SmartDashboard::PutNumber("D Gain", kD);
    // frc::SmartDashboard::PutNumber("I Zone", kIz);
    // frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    // frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    // frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    // frc::SmartDashboard::PutNumber("Set Rotations", 0);
  #endif

  #ifdef DEBUG_OTHER
    // frc::SmartDashboard::PutNumber("Ramp Increment Speed" , rampIncrementSpeed);
    // frc::SmartDashboard::PutNumber("Ramp Increment Rotation" , rampIncrementRotation);
    // frc::SmartDashboard::PutNumber("Max Speed" , maxSpeed);
    // frc::SmartDashboard::PutNumber("Max Rotation" , maxRotation);
  #endif
}

void Robot::RobotPeriodic() { // ERROR: warn user if there is no controler conected for input and disable all motors
  // ERROR: warn user if the Leonardo isnt conected
  useF310ForControl = f310.IsConnected(); // ERROR: warn user that the driver station is using the logitech for control inputs
  #ifdef DEBUG_CURRENT
  #pragma message("double mtr1Amps = PowerDistrobutionHub.GetCurrent(2);")
  double mtr1Amps = PowerDistrobutionHub.GetCurrent(2);
  double mtr2Amps = PowerDistrobutionHub.GetCurrent(1);
  double mtr3Amps = PowerDistrobutionHub.GetCurrent(0);
  double mtr4Amps = PowerDistrobutionHub.GetCurrent(13);
  double mtr5Amps = PowerDistrobutionHub.GetCurrent(14);
  double mtr6Amps = PowerDistrobutionHub.GetCurrent(15);
  double totalCurrent = PowerDistrobutionHub.GetTotalCurrent();
  // frc::SmartDashboard::PutNumber("MTR1 Current" , mtr1Amps);
  // frc::SmartDashboard::PutNumber("MTR2 Current" , mtr2Amps);
  // frc::SmartDashboard::PutNumber("MTR3 Current" , mtr3Amps);
  // frc::SmartDashboard::PutNumber("MTR4 Current" , mtr4Amps);
  // frc::SmartDashboard::PutNumber("MTR5 Current" , mtr5Amps);
  // frc::SmartDashboard::PutNumber("MTR6 Current" , mtr6Amps);
  // frc::SmartDashboard::PutNumber("Total Current", mtr6Amps);
  // frc::SmartDashboard::PutNumber("Total Current", mtr6Amps);
  if (maxCurrent < totalCurrent) {
    maxCurrent = totalCurrent;
    #pragma message("frc::SmartDashboard::PutNumber(" MAX Current ", maxCurrent);")
        // frc::SmartDashboard::PutNumber("MAX Current", maxCurrent);
  }
  #endif

  #ifdef DEBUG_OTHER
    // rampIncrementSpeed = frc::SmartDashboard::GetNumber("Ramp Increment Speed" , rampIncrementSpeed);
    // rampIncrementRotation = frc::SmartDashboard::GetNumber("Ramp Increment Rotation" , rampIncrementRotation);
    // maxSpeed = frc::SmartDashboard::GetNumber("Max Speed" , maxSpeed);
    // maxRotation = frc::SmartDashboard::GetNumber("Max Rotation" , maxRotation);
  #endif
}
#endif
#ifdef DEBUG_TUNABLE
void Robot::RobotInit() {
  frc::CameraServer::StartAutomaticCapture();

  LeftFront.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);
  LeftCenter.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);
  LeftRear.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);

  RightFront.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);
  RightCenter.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);
  RightRear.SetSecondaryCurrentLimit(MAX_CURRENT / NUM_OF_MOTORS);

  LeftFront.SetSmoothing(smoothingVal);
  LeftCenter.SetSmoothing(smoothingVal);
  LeftRear.SetSmoothing(smoothingVal);

  RightFront.SetSmoothing(smoothingVal);
  RightCenter.SetSmoothing(smoothingVal);
  RightRear.SetSmoothing(smoothingVal);

  LeftFront.SetInverted(false);
  LeftCenter.SetInverted(false);
  LeftRear.SetInverted(false);

  RightFront.SetInverted(true);
  RightCenter.SetInverted(true);
  RightRear.SetInverted(true);
  // frc::SmartDashboard::PutBoolean("DEBUG_TUNABLE", true);
}
void Robot::RobotPeriodic() {
  // double mtr1Amps = PowerDistrobutionHub.GetCurrent(2);
  // double mtr2Amps = PowerDistrobutionHub.GetCurrent(1);
  // double mtr3Amps = PowerDistrobutionHub.GetCurrent(0);
  // double mtr4Amps = PowerDistrobutionHub.GetCurrent(13);
  // double mtr5Amps = PowerDistrobutionHub.GetCurrent(14);
  // double mtr6Amps = PowerDistrobutionHub.GetCurrent(15);
  // double totalCurrent = PowerDistrobutionHub.GetTotalCurrent();
  // frc::SmartDashboard::PutNumber("MTR1 Current" , mtr1Amps);
  // frc::SmartDashboard::PutNumber("MTR2 Current" , mtr2Amps);
  // frc::SmartDashboard::PutNumber("MTR3 Current" , mtr3Amps);
  // frc::SmartDashboard::PutNumber("MTR4 Current" , mtr4Amps);
  // frc::SmartDashboard::PutNumber("MTR5 Current" , mtr5Amps);
  // frc::SmartDashboard::PutNumber("MTR6 Current" , mtr6Amps);
  // frc::SmartDashboard::PutNumber("Total Current", mtr6Amps);
  // frc::SmartDashboard::PutNumber("Total Current", mtr6Amps);
  // if (maxCurrent < totalCurrent) {
  //   maxCurrent = totalCurrent;
  //   frc::SmartDashboard::PutNumber("MAX Current", maxCurrent);
  // }

  // frc::SmartDashboard::PutNumber("LeftFrontMinForward"     ,   LeftFrontMinForward     );
  // frc::SmartDashboard::PutNumber("LeftCenterMinForward"    ,   LeftCenterMinForward    );
  // frc::SmartDashboard::PutNumber("LeftRearMinForward"      ,   LeftRearMinForward      );
  // frc::SmartDashboard::PutNumber("RightFrontMinForward"    ,   RightFrontMinForward    );
  // frc::SmartDashboard::PutNumber("RightCenterMinForward"   ,   RightCenterMinForward   );
  // frc::SmartDashboard::PutNumber("RightRearMinForward"     ,   RightRearMinForward     );

  // frc::SmartDashboard::PutNumber("LeftFrontMaxForward"     ,   LeftFrontMaxForward     );
  // frc::SmartDashboard::PutNumber("LeftCenterMaxForward"    ,   LeftCenterMaxForward    );
  // frc::SmartDashboard::PutNumber("LeftRearMaxForward"      ,   LeftRearMaxForward      );
  // frc::SmartDashboard::PutNumber("RightFrontMaxForward"    ,   RightFrontMaxForward    );
  // frc::SmartDashboard::PutNumber("RightCenterMaxForward"   ,   RightCenterMaxForward   );
  // frc::SmartDashboard::PutNumber("RightRearMaxForward"     ,   RightRearMaxForward     );

  // frc::SmartDashboard::PutNumber("LeftFrontMinBackward"     ,   LeftFrontMinBackward     );
  // frc::SmartDashboard::PutNumber("LeftCenterMinBackward"    ,   LeftCenterMinBackward    );
  // frc::SmartDashboard::PutNumber("LeftRearMinBackward"      ,   LeftRearMinBackward      );
  // frc::SmartDashboard::PutNumber("RightFrontMinBackward"    ,   RightFrontMinBackward    );
  // frc::SmartDashboard::PutNumber("RightCenterMinBackward"   ,   RightCenterMinBackward   );
  // frc::SmartDashboard::PutNumber("RightRearMinBackward"     ,   RightRearMinBackward     );

  // frc::SmartDashboard::PutNumber("LeftFrontMaxBackward"     ,   LeftFrontMaxBackward     );
  // frc::SmartDashboard::PutNumber("LeftCenterMaxBackward"    ,   LeftCenterMaxBackward    );
  // frc::SmartDashboard::PutNumber("LeftRearMaxBackward"      ,   LeftRearMaxBackward      );
  // frc::SmartDashboard::PutNumber("RightFrontMaxBackward"    ,   RightFrontMaxBackward    );
  // frc::SmartDashboard::PutNumber("RightCenterMaxBackward"   ,   RightCenterMaxBackward   );
  // frc::SmartDashboard::PutNumber("RightRearMaxBackward"     ,   RightRearMaxBackward     );

  // LeftFrontMinForward      =  frc::SmartDashboard::GetNumber("LeftFrontMinForward"     ,   LeftFrontMinForward     );
  // LeftCenterMinForward     =  frc::SmartDashboard::GetNumber("LeftCenterMinForward"    ,   LeftCenterMinForward    );
  // LeftRearMinForward       =  frc::SmartDashboard::GetNumber("LeftRearMinForward"      ,   LeftRearMinForward      );
  // RightFrontMinForward     =  frc::SmartDashboard::GetNumber("RightFrontMinForward"    ,   RightFrontMinForward    );
  // RightCenterMinForward    =  frc::SmartDashboard::GetNumber("RightCenterMinForward"   ,   RightCenterMinForward   );
  // RightRearMinForward      =  frc::SmartDashboard::GetNumber("RightRearMinForward"     ,   RightRearMinForward     );

  // LeftFrontMaxForward      =  frc::SmartDashboard::GetNumber("LeftFrontMaxForward"     ,   LeftFrontMaxForward     );
  // LeftCenterMaxForward     =  frc::SmartDashboard::GetNumber("LeftCenterMaxForward"    ,   LeftCenterMaxForward    );
  // LeftRearMaxForward       =  frc::SmartDashboard::GetNumber("LeftRearMaxForward"      ,   LeftRearMaxForward      );
  // RightFrontMaxForward     =  frc::SmartDashboard::GetNumber("RightFrontMaxForward"    ,   RightFrontMaxForward    );
  // RightCenterMaxForward    =  frc::SmartDashboard::GetNumber("RightCenterMaxForward"   ,   RightCenterMaxForward   );
  // RightRearMaxForward      =  frc::SmartDashboard::GetNumber("RightRearMaxForward"     ,   RightRearMaxForward     );

  // LeftFrontMinBackward     =  frc::SmartDashboard::GetNumber("LeftFrontMinBackward"     ,   LeftFrontMinBackward     );
  // LeftCenterMinBackward    =  frc::SmartDashboard::GetNumber("LeftCenterMinBackward"    ,   LeftCenterMinBackward    );
  // LeftRearMinBackward      =  frc::SmartDashboard::GetNumber("LeftRearMinBackward"      ,   LeftRearMinBackward      );
  // RightFrontMinBackward    =  frc::SmartDashboard::GetNumber("RightFrontMinBackward"    ,   RightFrontMinBackward    );
  // RightCenterMinBackward   =  frc::SmartDashboard::GetNumber("RightCenterMinBackward"   ,   RightCenterMinBackward   );
  // RightRearMinBackward     =  frc::SmartDashboard::GetNumber("RightRearMinBackward"     ,   RightRearMinBackward     );

  // LeftFrontMaxBackward      =  frc::SmartDashboard::GetNumber("LeftFrontMaxBackward"     ,   LeftFrontMaxBackward     );
  // LeftCenterMaxBackward     =  frc::SmartDashboard::GetNumber("LeftCenterMaxBackward"    ,   LeftCenterMaxBackward    );
  // LeftRearMaxBackward       =  frc::SmartDashboard::GetNumber("LeftRearMaxBackward"      ,   LeftRearMaxBackward      );
  // RightFrontMaxBackward     =  frc::SmartDashboard::GetNumber("RightFrontMaxBackward"    ,   RightFrontMaxBackward    );
  // RightCenterMaxBackward    =  frc::SmartDashboard::GetNumber("RightCenterMaxBackward"   ,   RightCenterMaxBackward   );
  // RightRearMaxBackward      =  frc::SmartDashboard::GetNumber("RightRearMaxBackward"     ,   RightRearMaxBackward     );

  // LeftFront.SetBounds     (LeftFrontMinForward,    LeftFrontMaxForward,    LeftFrontMinBackward,    LeftFrontMaxBackward);
  // LeftCenter.SetBounds    (LeftCenterMinForward,   LeftCenterMaxForward,   LeftCenterMinBackward,   LeftCenterMaxBackward);
  // LeftRear.SetBounds      (LeftRearMinForward,     LeftRearMaxForward,     LeftRearMinBackward,     LeftRearMaxBackward);

  // RightFront.SetBounds    (RightFrontMinForward,   RightFrontMaxForward,   RightFrontMinBackward,   RightFrontMaxBackward);
  // RightCenter.SetBounds   (RightCenterMinForward,  RightCenterMaxForward,  RightCenterMinBackward,  RightCenterMaxBackward);
  // RightRear.SetBounds     (RightRearMinForward,    RightRearMaxForward,    RightRearMinBackward,    RightRearMaxBackward);

  LeftFront.SetBounds(leftFrontParams [0], leftFrontParams [1], leftFrontParams [2], leftFrontParams [3]);
  LeftCenter.SetBounds(leftCenterParams [0], leftCenterParams [1], leftCenterParams [2], leftCenterParams [3]);
  LeftRear.SetBounds(leftRearParams [0], leftRearParams [1], leftRearParams [2], leftRearParams [3]);

  RightFront.SetBounds(rightFrontParams [0], rightFrontParams [1], rightFrontParams [2], rightFrontParams [3]);
  RightCenter.SetBounds(rightCenterParams [0], rightCenterParams [1], rightCenterParams [2], rightCenterParams [3]);
  RightRear.SetBounds(rightRearParams [0], rightRearParams [1], rightRearParams [2], rightRearParams [3]);
}
#endif

#ifdef DEBUG_OTHER
void Robot::AutonomousInit() {
  // set servos to center position
  panServo.SetAngle(90);
  tiltServo.SetAngle(90);
}

void Robot::AutonomousPeriodic() {
  if (direction == true) {
    // panServo.SetAngle(servoAngle++);
    // tiltServo.SetAngle(servoAngle++);
    std::cout << "true    ";
    servoAngle--;
    if (servoAngle < 15) {
      direction = !direction;
    }
  }
  else {
 // panServo.SetAngle(servoAngle--);
 // tiltServo.SetAngle(servoAngle--);
    std::cout << "false    ";
    servoAngle++;
    if (servoAngle > 160) {
      direction = !direction;
    }
  }
  std::cout << servoAngle << "\n";
  panServo.SetAngle(servoAngle);
  tiltServo.SetAngle(servoAngle);
  // panServo.SetAngle(90);                // TODO: add macros for max and min position for each servo to avoid crashes and loss of resolution
  // tiltServo.SetAngle(90);               // TODO: add macro for center position value (90)
  drivetrain.ArcadeDrive(0, 0, false); // TODO: add testing for controlling a relay on DIO 0 and DIO 1 for the headlights
  // panServo.SetAngle(90);
  // tiltServo.SetAngle(90);
}
#else
void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {
  panServo.SetAngle(PAN_CENTER);
  tiltServo.SetAngle(TILT_CENTER);
}
#endif

#ifndef DEBUG_TUNABLE
void Robot::TeleopInit() {
  // TODO: set motors to break mode
  // TODO: get first read of joystick positions
}
void Robot::TeleopPeriodic() {
  #ifdef DEBUG_OTHER
    // frc::SmartDashboard::PutBoolean("useF310ForControl", useF310ForControl);
  #endif
  #ifdef MAX_TEMP_COOLDOWN
  bool tooHot = maxTempCuttoffPin.Get();
  if (!tooHot) {
    #endif
    if (useF310ForControl) {
      // if (true) {
      /*TODO: create global variable that starts false and is set true after first run through this loop, so that the first run through this loop can set all variables to known value*/
      float leftStickXAxisPos = f310.GetRawAxis(F310_LEFT_STICK_X_AXIS);
      float leftStickYAxisPos = f310.GetRawAxis(F310_LEFT_STICK_Y_AXIS);
      #ifdef CAMERA_PAN_TILT
      float rightStickXAxisPos = f310.GetRawAxis(F310_RIGHT_STICK_X_AXIS);
      float rightStickYAxisPos = f310.GetRawAxis(F310_RIGHT_STICK_Y_AXIS);
      #endif
      bool headlightOutput = f310.GetRawButton(F310_Y_BUTTON);
      bool surfaceLightOutput = f310.GetRawButton(F310_A_BUTTON);
      Headlight.Set(headlightOutput);
      SurfaceLight.Set(surfaceLightOutput);
      float rotation = leftStickXAxisPos;
      float speed = leftStickYAxisPos;

      #ifdef CAMERA_PAN_TILT
            // panAngle = RampVal(panAngle, rightStickXAxisPos, 5);
            // tiltAngle = RampVal(tiltAngle, rightStickYAxisPos, 5);
      panAngle = rightStickXAxisPos;
      tiltAngle = rightStickYAxisPos;
      #endif

      #ifdef MAX_SPEED_FIX
            // #ifdef FEATURE_UNDER_DEVELOPMENT
      newSpeed = RampVal(map(newSpeed, -1, 1, maxSpeed * -1, maxSpeed), speed, rampIncrementSpeed);
      newRotation = RampVal(map(newRotation, -1, 1, maxRotation * -1, maxRotation), rotation, rampIncrementRotation);
      #else
      newSpeed = RampVal(newSpeed, speed, rampIncrementSpeed);
      newRotation = RampVal(newRotation, rotation, rampIncrementRotation);
      #endif

      #ifdef DEBUG_OTHER
            // frc::SmartDashboard::PutNumber("Rotation Out" , newRotation);
            // frc::SmartDashboard::PutNumber("Rotation Target" , rotation);
            // frc::SmartDashboard::PutNumber("Speed Output" , newSpeed);
            // frc::SmartDashboard::PutNumber("Speed Target" , speed);
      #endif

            // drivetrain.ArcadeDrive(MAX_MOTOR_OUTPUT * newSpeed, MAX_ROTATION_OUTPUT * newRotation, false);
      drivetrain.ArcadeDrive(maxSpeed * newSpeed, maxRotation * newRotation * -1, false);
      // panServo.SetAngle(panAngle);
      // std::cout << "Pan angle: " << panAngle;
      // tiltServo.SetAngle(tiltAngle);
      // std::cout << "   Tilt angle: "<< tiltAngle << "\n";
      // dbg(panAngleRequest)
      // dbg(" 1  ");
      // dbg(tiltAngle);       // TODO: MACROS NOT WORKING
      // dbg("   ");
      // dbg(panAngle);
      // dbg("   ");
      // dbgln(tiltAngle);
    }
    else {
      float steeringJoystickPos = arduinoLeonardo.GetRawAxis(LEONARDO_STEERING_JOYSTICK);
      float speedJoystickPos = arduinoLeonardo.GetRawAxis(LEONARDO_SPEED_JOYSTICK);
      #ifdef CAMERA_PAN_TILT
      float panAngleRequest = arduinoLeonardo.GetRawAxis(LEONARDO_CAMERA_PAN);
      float tiltAngleRequest = arduinoLeonardo.GetRawAxis(LEONARDO_CAMERA_TILT);
      #endif
      float speed = speedJoystickPos;
      float rotation = steeringJoystickPos;

      #ifdef CAMERA_PAN_TILT
            // panAngle = RampVal(panAngle, panAngleRequest, 20);
            // tiltAngle = RampVal(tiltAngle, tiltAngleRequest, 20);
      panAngle = panAngleRequest;
      tiltAngle = tiltAngleRequest;
      #endif

      drivetrain.ArcadeDrive(-1 * speed, -1 * rotation, false);
      // panServo.SetAngle(panAngle);
      // std::cout << "Pan angle: " << panAngle;
      // tiltServo.SetAngle(tiltAngle);
      // std::cout << "   Tilt angle: "<< tiltAngle << "\n";
      // dbg(panAngleRequest);
      // dbg(" 2  ");
      // dbg(tiltAngle);      // TODO: MACROS NOT WORKING
      // dbg("   ");
      // dbg(panAngle);
      // dbg("   ");
      // dbgln(tiltAngle);

      bool headlightOutput = arduinoLeonardo.GetRawButton(LEONARDO_HEADLIGHT_SWICH);
      bool surfaceLightOutput = arduinoLeonardo.GetRawButton(LEONARDO_SURFACELIGHT_SWICH);
      Headlight.Set(headlightOutput);
      SurfaceLight.Set(surfaceLightOutput);
      // headlightOutput = arduinoLeonardo.GetRawButton(LEONARDO_UPDATE_INFO_BUTTON);

      if (infoUpdateButtonState != prevInfoUpdateButtonState) { // ensures user cant hold the button down and constantly update the data
        if (infoUpdateButtonState) {
          requestData = !requestData;
        }
        prevInfoUpdateButtonState = infoUpdateButtonState;
      }
    }
    #ifdef MAX_TEMP_COOLDOWN
        // frc::SmartDashboard::PutString("Temprature", "GOOD");
        // frc::SmartDashboard::PutBoolean("Status", true);
  } // closing bracket for if statement that checks arduino thats monitoring temps
  else {
    // frc::SmartDashboard::PutString("Temprature", "!!OVERHEATED!!");
    // frc::SmartDashboard::PutBoolean("Status", false);
  }
  #endif

  #ifdef CAMERA_PAN_TILT
  panAngle = map(panAngle, 1, -1, MIN_PAN_ANGLE, MAX_PAN_ANGLE);
  tiltAngle = map(tiltAngle, -1, 1, MIN_TILT_ANGLE, MAX_TILT_ANGLE);
  panServo.SetAngle(panAngle);
  std::cout << "Pan angle: " << panAngle;
  tiltServo.SetAngle(tiltAngle);
  std::cout << "   Tilt angle: " << tiltAngle << "\n";
  #else
  panServo.SetAngle(PAN_CENTER);
  tiltServo.SetAngle(TILT_CENTER);
  #endif

  #ifdef FEATURE_UNDER_DEVELOPMENT
  SendData();
  #endif
}
#else
void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  // tunableDrivetrain.Drive(f310.GetRawAxis(F310_LEFT_STICK_Y_AXIS), f310.GetRawAxis(F310_RIGHT_STICK_Y_AXIS) * -1);

  // double xJoyPos = f310.GetRawAxis(F310_RIGHT_STICK_X_AXIS);
  // double yJoyPos = f310.GetRawAxis(F310_RIGHT_STICK_Y_AXIS);

  // double xJoyPos = GetJoyWithDZ(f310.GetRawAxis(F310_RIGHT_STICK_X_AXIS), .03, -.03);
  // double yJoyPos = GetJoyWithDZ(f310.GetRawAxis(F310_RIGHT_STICK_Y_AXIS), .03, -.03);

  // double xJoyPos = GetJoyWithDZ(PS4.GetRawAxis(1), .03, -.03);
  // double yJoyPos = GetJoyWithDZ(PS4.GetRawAxis(2), .03, -.03);

  float xJoyPos = GetJoyWithDZ(arduinoLeonardo.GetRawAxis(LEONARDO_STEERING_JOYSTICK), .03, -.03);
  float yJoyPos = GetJoyWithDZ(arduinoLeonardo.GetRawAxis(LEONARDO_SPEED_JOYSTICK), .03, -.03);

  frc::SmartDashboard::PutNumber("xJoyPos", xJoyPos);
  frc::SmartDashboard::PutNumber("yJoyPos", -yJoyPos);

  // double xJoyPos = GetJoyWithDZ(.21, .2, -.2);
  // double yJoyPos = GetJoyWithDZ(.21, .2, -.2);

  // setDrivetrain(xJoyPos, yJoyPos, 1);
  setDrivetrainNoLimmit(xJoyPos, yJoyPos, 1);

  float panAngleRequest = arduinoLeonardo.GetRawAxis(LEONARDO_CAMERA_PAN);
  float tiltAngleRequest = arduinoLeonardo.GetRawAxis(LEONARDO_CAMERA_TILT);

  panAngle = RampVal(panAngle, panAngleRequest, .005);
  tiltAngle = RampVal(tiltAngle, tiltAngleRequest, .005);

  panAngle = panAngleRequest;
  tiltAngle = tiltAngleRequest;

  panAngle = map(panAngle, 1, -1, MIN_PAN_ANGLE, MAX_PAN_ANGLE);
  tiltAngle = map(tiltAngle, -1, 1, MIN_TILT_ANGLE, MAX_TILT_ANGLE);
  panServo.SetAngle(panAngle);
  tiltServo.SetAngle(tiltAngle);
}
#endif

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

double currentVelocity;
float currentMotorSetpoint;
float leftStickYAxisPos;
float speed;

#ifdef DEBUG_OTHER
#ifdef FEATURE_UNDER_DEVELOPMENT
void Robot::directSpeedControl() {
  double currentVelocity = LeftFrontEncoder.GetVelocity();
  float currentMotorSetpoint;
  float leftStickYAxisPos = f310.GetRawAxis(F310_LEFT_STICK_Y_AXIS);
  float speed = leftStickYAxisPos;
}
#endif
#endif

#ifdef DEBUG_OTHER
void Robot::TestInit() {
  motorSpeed = 0;
  motorsDrivingForward = true;
}
void Robot::TestPeriodic() {
  #ifdef FEATURE_UNDER_DEVELOPMENT
    // drivetrainMotorTest();
    // double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    // double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    // double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    // double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    // double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    // double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    // double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    // double velocity = frc::SmartDashboard::GetNumber("Set Velocity", 0);

  if (p != kP) {
    LeftFrontPIDController.SetP(p);
    kP = p;
  }
  if (i != kI) {
    LeftFrontPIDController.SetI(i);
    kI = i;
  }
  if (d != kD) {
    LeftFrontPIDController.SetD(d);
    kD = d;
  }
  if (iz != kIz) {
    LeftFrontPIDController.SetIZone(iz);
    kIz = iz;
  }
  if (ff != kFF) {
    LeftFrontPIDController.SetFF(ff);
    kFF = ff;
  }
  if ((max != kMaxOutput) || (min != kMinOutput)) {
    LeftFrontPIDController.SetOutputRange(min, max);
    kMinOutput = min;
    kMaxOutput = max;
  }
  #endif

    /**
     * PIDController objects are commanded to a set point using the
     * SetReference() method.
     *
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     *
     * The second parameter is the control type can be set to one of four
     * parameters:
     *  rev::CANSparkMax::ControlType::kDutyCycle
     *  rev::CANSparkMax::ControlType::kPosition
     *  rev::CANSparkMax::ControlType::kVelocity
     *  rev::CANSparkMax::ControlType::kVoltage
     */

  #ifdef FEATURE_UNDER_DEVELOPMENT
  LeftFrontPIDController.SetReference(velocity, rev::CANSparkMax::ControlType::kVelocity);

  // frc::SmartDashboard::PutNumber("SetPoint", velocity);
  // frc::SmartDashboard::PutNumber("LeftFrontCurrentVelocity", LeftFrontEncoder.GetVelocity());
  #endif
}
#else
void Robot::TestInit() {}
void Robot::TestPeriodic() {}
#endif

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
