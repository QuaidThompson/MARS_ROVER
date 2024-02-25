// use defines to create different words that make more sense

//motor types

#define BRUSHED          rev::CANSparkMaxLowLevel::MotorType::kBrushed
// #define Brushed          rev::CANSparkMaxLowLevel::MotorType::kBrushed
// #define brushed          rev::CANSparkMaxLowLevel::MotorType::kBrushed

#define BRUSHLESS        rev::CANSparkMaxLowLevel::MotorType::kBrushless
// #define Brushless        rev::CANSparkMaxLowLevel::MotorType::kBrushless
// #define brushless        rev::CANSparkMaxLowLevel::MotorType::kBrushless

// #ifdef FEATURE_UNDER_DEVELOPMENT
    #define QUADRATURE       rev::CANEncoder::EncoderType::kQuadrature
    #define Quadrature       rev::CANEncoder::EncoderType::kQuadrature
    #define quadrature       rev::CANEncoder::EncoderType::kQuadrature
// #endif