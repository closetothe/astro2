// // // // // // // // // // // // // // // // // // //
//   REPLACE WITH YOUR HARDWARE-SPECIFIC PARAMETERS   //
// // // // // // // // // // // // // // // // // // //

// H-bridge inputs
// Motor 1
#define ENA 10
#define IN1 12
#define IN2 13
// Motor 2
#define ENB 3
#define IN3 4
#define IN4 5

// LEDs
#define LED_LEFT A4
#define LED_RIGHT A5

// Min and max velocity
// (in m/s)
#define VMIN 0.0
#define VMAX 0.5

// PWM (i.e. voltage) limit
#define RLIMIT 200
#define LLIMIT 200

// Experimental coefficients
// pwm = a*(rpm) + b
#define RIGHTA 800
#define RIGHTB -27.8
#define LEFTA 800
#define LEFTB -27.8

// Motor deadbands
#define LEFTDB 30
#define RIGHTDB 30

// Wheel diameters
// (in meters)
#define LDIA 0.1524
#define RDIA 0.1524