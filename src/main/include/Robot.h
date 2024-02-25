// Robot.h

#pragma once


#include "keyWordConversions.h"
#include "CAN_ID_DEFFINITIONS.cpp"
// #include "configurations.h"

#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/GenericHID.h>
#include <frc/TimedRobot.h>
#include <frc/DigitalOutput.h>
#include <frc/DigitalInput.h>
#include <frc/Servo.h>
#include <cameraserver/CameraServer.h>
#include <frc/DriverStation.h>
#include <frc/PowerDistribution.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANEncoder.h>

#include "TunableMotorController.h"
// #include "TunableMotorControllerGroup.h"
// #include "TunableDrivetrain.h"



#define TICKS_PER_REVOLUTION 80



class Robot : public frc::TimedRobot {
    public:
    
    //////////////////////////////////
    /*motor controller declaarations*/
    #ifdef CUSTOM_DRIVETRAIN
        TunableMotorController LeftFront    {LEFT_FRONT_MOTOR_CONTROLLER_CAN_ID,    TunableMotorController::p_MotorType::Brushed};
        TunableMotorController LeftCenter   {LEFT_CENTER_MOTOR_CONTROLLER_CAN_ID,   TunableMotorController::p_MotorType::Brushed};
        TunableMotorController LeftRear     {LEFT_REAR_MOTOR_CONTROLLER_CAN_ID,     TunableMotorController::p_MotorType::Brushed};
        TunableMotorController RightFront   {RIGHT_FRONT_MOTOR_CONTROLLER_CAN_ID,   TunableMotorController::p_MotorType::Brushed};
        TunableMotorController RightCenter  {RIGHT_CENTER_MOTOR_CONTROLLER_CAN_ID,  TunableMotorController::p_MotorType::Brushed};
        TunableMotorController RightRear    {RIGHT_REAR_MOTOR_CONTROLLER_CAN_ID,    TunableMotorController::p_MotorType::Brushed};

        // TunableMotorControllerGroup LeftTunable;
        // TunableMotorControllerGroup RightTunable;

        // TunableDrivetrain tunableDrivetrain;

        
    #endif


    #ifndef CUSTOM_DRIVETRAIN
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
    #endif

    /*motor controller declaarations*/
    //////////////////////////////////

    #ifdef FEATURE_UNDER_DEVELOPMENT
        //////////////////
        /*Encoder Settup*/
        rev::SparkMaxRelativeEncoder LeftFrontEncoder = LeftFront.GetEncoder(Quadrature,TICKS_PER_REVOLUTION);
        /*Encoder Settup*/
        //////////////////

        ///////////////////
        /*PID Controllers*/
        rev::SparkMaxPIDController LeftFrontPIDController = LeftFront.GetPIDController();
        /*PID Controllers*/
        ///////////////////
    #endif

    ///////////////////////////
    /*controller declarations*/
    frc::GenericHID arduinoLeonardo {0};
    frc::GenericHID f310 {1};
    /*controller declarations*/
    ///////////////////////////

    /////////////////////////////////////////
    /*Power Distrobution Pannel Declaration*/
    frc::PowerDistribution PowerDistrobutionHub{0, frc::PowerDistribution::ModuleType::kCTRE};
    /*Power Distrobution Pannel Declaration*/
    /////////////////////////////////////////

    //////////////////////
    /*light declarations*/
    frc::DigitalOutput Headlight{0};
    frc::DigitalOutput SurfaceLight{1};
    /*light declarations*/
    //////////////////////

    //////////////////////
    /*servo declarations*/
    frc::Servo panServo {0};
    frc::Servo tiltServo {1};
    /*servo declarations*/
    //////////////////////

    #ifdef MAX_TEMP_COOLDOWN
    ////////////////////////
    /*max temp cuttoff pin*/
    frc::DigitalInput maxTempCuttoffPin{9};
    /*max temp cuttoff pin*/
    ////////////////////////
    #endif

    #ifdef DEBUG_OTHER
        void drivetrainMotorTest();
        void directSpeedControl();
    #endif

    void setDrivetrain(double rateOfTurn, double speed, double mixConstant, bool zeroTurn);
    double GetJoyWithDZ(double joystickVal, double minPosVal, double maxNegVal);


    void RobotInit() override;
    void RobotPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void SimulationInit() override;
    void SimulationPeriodic() override;

    private:
};