// Constants - general
#define MON_INTERVAL    1       // interval at which monitor task is called, in seconds
#define FLASH_EN        0

// Constants for motor drive
#define PWM_PERIOD      50       // in micro-seconds, or 20khz
#define MTR_EN          1
#define DIR_CW          0
#define STARTUP_DC      0.05f     // duty cycle required to start the motor
#define DC_MAX          1           // dutycycle max
#define DC_MIN          0.10           // dutycycle max
#define SET_SPEED       200     // desired speed, in RPM

// Constants for PID
#define PID_INTERVAL    0.20            // interval at which PID loop at, in seconds
#define Kp              0.00025         // Kp - proportional gain
#define Ki              0.00004         // Ki - integratal gain
#define Kd              0.00001              // Ki - integratal gain
#define DEADBAND        1           // Acceptable error, in rpm

// to enable testing
//#define RAMP_TEST_ON

#include "mbed.h"

// Defining I/O pins
DigitalOut      myled   ( LED1 );
Serial          pc      ( USBTX, USBRX );
DigitalOut      dir_o   ( p18 );        // orange
DigitalOut      start_o ( p19 );        // white
PwmOut          pwm_o   ( p22 );        // blue
InterruptIn     tach_i  ( p21 );        // yellow

// Defining timers and tickers
Timer           spd_timer;              // timer used to measure period of tach_i input
Ticker          mon_tick;
Ticker          PID_tick;


float       dc = STARTUP_DC;
int         T_tach = 0;     // period of tach_i
float       actual_spd;     // measured speed
float       set_spd = SET_SPEED;     // setpoint
float       spd_error = 0;          // error
float       intg_error = 0;         // integral error
float       deriv = 0;
float       prev_error = 0;
float       pid_out = 0;
       

///////////////////////////////////////////////////////////////////////////////
// GetSpeed
// This function is called upon each rising edge of tach_i input.
// The spd_timer measures every period of the tach input, which represents the speed.
///////////////////////////////////////////////////////////////////////////////
void GetSpeed() {
    spd_timer.stop();       // stop the timer
    T_tach = spd_timer.read_us();       // store the timer value
    spd_timer.reset();          // reset and restart the timer
    spd_timer.start();
}


///////////////////////////////////////////////////////////////////////////////
// PID Control Loop
///////////////////////////////////////////////////////////////////////////////
void PID_task() {
    //pc.printf( "T_tach = %d\r\n", T_tach );
    actual_spd = 10000000 / ( 3 * T_tach );     // Converts tach period to speed
    //pc.printf( "actual_spd = %.2f rpm\r\n", actual_spd );
    
  
/* Pseudo code from wikipedia

     previous_error = 0
     integral = 0 
   start:
     error = setpoint – actual
     integral = integral + error*dt
     derivative = (error - previous_error)/dt
     output = Kp*error + Ki*integral + Kd*derivative
     previous_error = error
     wait(dt)
     goto start
*/

    // calculate diff between actual and setpoint
    spd_error = set_spd - actual_spd;
    
    // don't process PID if actual speed is within the acceptable range
    if( abs(spd_error) < DEADBAND ) {
//        pc.printf(" err=%.2f \r\n", spd_error );
        spd_error = 0;
        pid_out = 0;
    }
    else {
        // calculate integral error, this is the error over time
        intg_error = intg_error + spd_error * PID_INTERVAL;
        
        // calculate derivative, this is the amount of change from the last time checked
        deriv = ( spd_error - prev_error ) / PID_INTERVAL;
        
        // calculate new output
        pid_out = Kp * spd_error + Ki * intg_error + Kd * deriv;
//        pc.printf( "pid_out=%.6f\r\n\n", pid_out );
    
        // remember error
        prev_error = spd_error;

        // Update dutycycle by incorporating pid_out
        if( (dc + pid_out <= DC_MAX) && (dc + pid_out >= DC_MIN) ) {
            dc += pid_out;
    //        pc.printf( "dc updated \r\n");
            // write new pwm dutycycle
            pwm_o.write( dc );
        }    
    }        
}


///////////////////////////////////////////////////////////////////////////////
// Monitor 
///////////////////////////////////////////////////////////////////////////////
void mon_task() {
    pc.printf( "actual=%.0f rpm, setpt=%.0f rpm, err=%.1f, intg_err=%.1f, pid_out=%.3f, dc=%.1f \r\n", actual_spd, set_spd, spd_error, intg_error, pid_out, dc*100 );
}


///////////////////////////////////////////////////////////////////////////////
// Main
///////////////////////////////////////////////////////////////////////////////
int main() {
    
    pc.printf("\r\n\r\n///////////////////////////////\r\n");
    pc.printf("//  PID for BLDC Motor Control\r\n");
    pc.printf("///////////////////////////////\r\n\n");

    // initialize outputs
    start_o = !MTR_EN;
    dir_o = DIR_CW;
    wait_ms( 100 );
    
    // specify period and dutycycle
    pwm_o.period_us( PWM_PERIOD );
    pwm_o.write( dc );
    wait( 1 );

    // enable the motor
    start_o = MTR_EN;
    
    // setup interrupt on tach_i input
    tach_i.rise( &GetSpeed );
    
    // assign timers
    PID_tick.attach( &PID_task, PID_INTERVAL );       // attach PID_task to tick
    mon_tick.attach( &mon_task, MON_INTERVAL );        // attach mon_task to tick
    
    while( 1 ) {

        #ifdef RAMP_TEST_ON
            // Open loop ramp test to verify motor functionality, PID not used
            if( dc < 1.00 ) {
                dc += 0.10;
                pwm_o.write( dc );
            }
            else {
                start_o = !MTR_EN;
            }
        #endif

    }   // while

}   // main
