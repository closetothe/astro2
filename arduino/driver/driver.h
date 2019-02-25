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
#define VMIN 0.00
#define VMAX 0.33

// PWM (i.e. voltage) limit
#define RLIMIT 125
#define LLIMIT 125

// Experimental coefficients
// pwm = a*(v) + b
#define RIGHTA 355.41
#define RIGHTB 11.987
#define LEFTA 364.9
#define LEFTB 10.571

// Motor deadbands
#define LEFTDB 40
#define RIGHTDB 40

// Wheel diameters
// (in meters)
#define LDIA 0.1524
#define RDIA 0.1524
