// TunableMotorController.cpp


#ifndef TunableMotorController_cpp
#define TunableMotorController_cpp

    /**
     * This Libbrary is designed to make use
     * of the individual drivetrain control
     * style, but allow for "tuning" the
     * dead-zone and max speed of brushed
     * motors. The use should be the same as
     * the WPILib provided API for motor
     * controller groups and drivetrain
     * groups, as well as allow tuning the
     * paramaters for each motor durring
     * initialization.
    */


    #include <rev/CANSparkMax.h>
    #include <frc/motorcontrol/MotorControllerGroup.h>
    #include <frc/drive/DifferentialDrive.h>
    #include <rev/CANSparkMaxLowLevel.h>

    class TunableMotorController {
        private:
            rev::CANSparkMax* p_motor;  // Declare the motor as a member variable
            double p_minForward = 0;
            double p_maxForward = 1;
            double p_minBackward = 0;
            double p_maxBackward = -1;
            double p_targetMotorSpeed = 0;
            double p_currentSpeed = 0;
            double p_rampIncrement = 1;
            bool   p_invert = false;
            float p_map(float input, float inA, float inb, float outA, float outB);      // map function for easy conversion of input to output values
            float p_RampVal(float currentVal, float targetVal, float rampIncriment);
        public:
            enum class p_MotorType { Brushed = 0, Brushless = 1 };
            explicit TunableMotorController(int m_MotorControllerID, p_MotorType m_MotorType);
            void Begin(int m_MotorControllerID, p_MotorType m_MotorType);
            void SetBounds(double minForward, double maxForward, double minBackward, double maxBackward);
            void SetSmoothing(double rampIncrement);
            void Set(double percentOutput);
            void SetInverted(bool invert);
            void SetSecondaryCurrentLimit(double maxAmps);
    };



#endif