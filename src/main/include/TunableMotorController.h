// TunableMotorController.h

#ifndef TunableMotorController_h
#define TunableMotorController_h


    /**
     * This Library is designed to make use
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



    #include <TunableMotorController.cpp>
    #include <iostream>








    TunableMotorController::TunableMotorController(int m_MotorControllerID, p_MotorType m_MotorType) {
        if (m_MotorType == p_MotorType::Brushed) {
            p_motor = new rev::CANSparkMax(m_MotorControllerID, rev::CANSparkMaxLowLevel::MotorType::kBrushed);
        } else if (m_MotorType == p_MotorType::Brushless) {
            p_motor = new rev::CANSparkMax(m_MotorControllerID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        } else {
            std::cout << "Invalid motor type\n";
            // Handle the error appropriately
        }
    }

    float TunableMotorController::p_map(float input, float inA, float inb, float outA, float outB) {      // map function for easy conversion of input to output values
        float m_output = outA + ((outB - outA) / (inb -inA)) * (input - inA);
        return m_output;
    }

    float TunableMotorController::p_RampVal(float currentVal, float targetVal, float rampIncriment) {
        if (currentVal != targetVal){            // do we need to ramp?
            if (currentVal > targetVal) {          // if we are ramping down
                float f = currentVal - targetVal;
                if (f > rampIncriment) {
                    currentVal -= rampIncriment;
                    return currentVal;
                } else {
                    return targetVal;
                }
            } else if (currentVal < targetVal) {   // if we are ramping up
                float f = targetVal - currentVal;
                if (f > rampIncriment) {
                    currentVal += rampIncriment;
                    return currentVal;
                } else {
                    return targetVal;
                }
            } else {
                std::cout << "!!! N.F.G. !!!\n";    // shouldnt ever get here
                return 0;
            }
        } else {          // we are already at the intended value
            return targetVal;
        }
    }

    void TunableMotorController::Begin(int m_MotorControllerID, p_MotorType m_MotorType) {
        if (m_MotorType == p_MotorType::Brushed) {
            p_motor = new rev::CANSparkMax(m_MotorControllerID, rev::CANSparkMaxLowLevel::MotorType::kBrushed);
        } else if (m_MotorType == p_MotorType::Brushless) {
            p_motor = new rev::CANSparkMax(m_MotorControllerID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        } else {
            std::cout << "Invalid motor type\n";
            // Handle the error appropriately
        }
    }

    void TunableMotorController::SetInverted(bool invert) {
        p_invert = invert;
    }

    void TunableMotorController::SetSmoothing(double rampIncrement) {
        p_rampIncrement = rampIncrement;
    }

    void TunableMotorController::Set(double percentOutput) {
        double pl_percentOutput = percentOutput;
        if (p_invert) {
            pl_percentOutput = percentOutput * -1;
        }

        if (p_motor) {
            if (pl_percentOutput > 0) {  // forward
                p_currentSpeed = p_RampVal(p_currentSpeed , p_map(pl_percentOutput, 0, 1, p_minForward, p_maxForward), p_rampIncrement);
                p_motor->Set(p_currentSpeed);
            } else if (pl_percentOutput < 0) {  // backward
                p_currentSpeed = p_RampVal(p_currentSpeed , p_map(pl_percentOutput, 0, -1, p_minBackward, p_maxBackward), p_rampIncrement);
                p_motor->Set(p_currentSpeed);
            } else {  // stopped
                p_motor->Set(0);
            }
            
        } else {
            // Handle the error appropriately
            std::cout << "Motor not initialized\n";
        }
    }

    void TunableMotorController::SetBounds(double minForward, double maxForward, double minBackward, double maxBackward) {
        p_minForward = minForward;
        p_maxForward = maxForward;
        p_minBackward = minBackward;
        p_maxBackward = maxBackward;
    }

    void TunableMotorController::SetSecondaryCurrentLimit(double maxAmps) {
        p_motor->SetSecondaryCurrentLimit(maxAmps);
    }

    void TunableMotorController::SetIdleMode(rev::CANSparkMax::IdleMode mode) {
        if (p_motor) {
            p_motor->SetIdleMode(mode);
        } else {
            std::cout << "Motor not initialized\n";
            // Handle the error appropriately
        }
    }




#endif