/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 470 ohm resistor ---> VGA Green
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - GPIO 21 ---> 330 ohm resistor ---> VGA Red
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */ 

// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"

// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"

// Include custom libraries
#include "vga16_graphics_v2.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"

#define BUTTON_PIN 15

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// character array
char screentext[40];

// draw speed
int threshold = 10 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// Some parameters for PWM
#define WRAPVAL 6000 
#define CLKDIV  25.0f
uint slice_num ;

// GPIO we're using for PWM
#define PWM_OUT 4

// control variables PWM duty cycle
volatile int control;
volatile int old_control;
volatile int filtered_control;

// complementary filter variables
volatile fix15 complementary_angle;
static  fix15 accel_angle;
static  fix15 gyro_angle_delta;
static  fix15 gyro_weight = float2fix15(0.99);
static  fix15 accel_weight = float2fix15(0.01);

// low-pass filtered accelerometer values
static fix15 filtered_az = 0;
static fix15 filtered_ay = 0;

// pid gain constants
static float Kp = 250.0f; 
static float Kd = 100.0f;
static float Ki = 0.02f;

// target angle and pid tracking
static float angle_in_deg = -92.0f;
static float prev_error = 0.0f;
static float accum_error_dt = 0.0f;
static float error = 0.0;

// ==================================================
// === PID Controller
// ==================================================
float pid(float angle_setpoint_deg, fix15 complementary_angle) {
    // compute current angle error (deg)
    error = angle_setpoint_deg - fix2float15(complementary_angle);
    
    // integrate error (for I)
    accum_error_dt = accum_error_dt + (error * zeropt001); 

    // clamp integral term to prevent wind-up
    if (accum_error_dt > 160000.0f) {
        accum_error_dt = 160000.0f;
    }
    else if (accum_error_dt < -160000.0f){
        accum_error_dt = -160000.0f;
    }
    
    // compute PID output
    // proportional + derivative (gyro rate) + integral
    float out = (Kp * error) + (fix2float15(gyro[0]) * Kd) + (Ki * accum_error_dt); 

    prev_error = error;
    return out;
}

// ==================================================
// === PWM ISR
// ==================================================
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_OUT));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // low pass accelerometer
    filtered_az = filtered_az + ((acceleration[2] - filtered_az) >> 4); 
    filtered_ay = filtered_ay + ((acceleration[1] - filtered_ay) >> 4);

    // NO SMALL ANGLE APPROXIMATION
    accel_angle = -multfix15(float2fix15(atan2( fix2float15(filtered_ay), fix2float15(filtered_az) ) ),oneeightyoverpi);

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    gyro_angle_delta = multfix15(-gyro[0], zeropt001);

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle = multfix15(complementary_angle +  gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    // get motor command from pid()
    control = (int)pid(angle_in_deg, complementary_angle);
    control = min(max((int)control, 0), 4000); // clamping

    // smooth motor control output
    filtered_control = (int)(filtered_control + (((control) - filtered_control) >> 5)); 
    
    // Update duty cycle
    if (control!=old_control) {
        old_control = control ;
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control);
    }
    
    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);

}

// ==================================================
// === Button thread
// ==================================================
static PT_THREAD(protothread_button(struct pt *pt))
{
    PT_BEGIN(pt);
    // config
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);   

    static bool prev = true; // start in "not pressed"
    static bool sequence_done = false;
    static bool sequence_armed = false;
    bool holding = false;

    // initialize motor off
    control = 0;
    filtered_control = 0;
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0); 

    while (1) {
        bool now = gpio_get(BUTTON_PIN);  
        
        // detect button press
        if (prev && !now) {
            holding = true;
            sequence_armed = true;
            angle_in_deg = -92.0f;
            control = 0;
            filtered_control = 0;
            pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);  
        }
        // detect button release
        else if (!prev && now) {
            holding = false;
            if (sequence_armed) { // run angle sequence 
                sequence_armed = false;
                angle_in_deg = 0.0f;
                PT_YIELD_usec(5000000);
                angle_in_deg = 30.0f;
                PT_YIELD_usec(5000000);
                angle_in_deg = -30.0f;
                PT_YIELD_usec(5000000);
                angle_in_deg = 0.0f;
            
            }
        }
        if (holding) {
            control = 0;
            filtered_control = 0;
            pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
            angle_in_deg = -92.0f;
        }
        prev = now;

        PT_YIELD_usec(100000);   
    }

    PT_END(pt);
}

// ==================================================
// === VGA Thread
// ==================================================
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 500. ;
    static float NewRange = 150. ; 
    static float OldMin = -250. ;
    static float OldMax = 250. ;

    // Control rate of drawing
    static int throttle ;
    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw top plot (Angle in degrees)
    drawHLine(75, 230, 5, RED) ;
    drawHLine(75, 155, 5, RED) ;
    drawHLine(75, 80,  5, RED) ;
    drawVLine(80, 80, 150, RED) ;

    sprintf(screentext, "0 deg") ;
    setCursor(35, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+90") ;
    setCursor(40, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-90") ;
    setCursor(40, 225) ;
    writeString(screentext) ;
    

    // Draw bottom plot for motor command
    drawHLine(75, 430, 5, CYAN);
    drawHLine(75, 355, 5, CYAN);
    drawHLine(75, 280, 5, CYAN);
    drawVLine(80, 280, 150, CYAN);

    sprintf(screentext, "0");
    setCursor(50, 425);
    writeString(screentext);
    sprintf(screentext, "%d", WRAPVAL/4);
    setCursor(40, 355);
    writeString(screentext);
    sprintf(screentext, "%d", WRAPVAL/2);
    setCursor(40, 280);
    writeString(screentext);


    
    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // --- Bottom plot ---
            static float smooth_val = 0.0f;
            static int prev_y_motor = 430;

            const float alpha = 0.1f;  // smoothing constant

            // lowpass filter 
            smooth_val = smooth_val * (1 - alpha) + filtered_control * alpha;

            int y_motor = 430 - (int)(150.0f * (smooth_val / (float)WRAPVAL));
            if (y_motor < 0) { // clamping
                y_motor = 0;
            }
            if (y_motor > 479) {
                y_motor = 479;
            }
            // draw line between previous and current positions
            int step = (y_motor > prev_y_motor) ? 1 : -1;
            for (int i = prev_y_motor; i != y_motor + step; i += step)
                drawPixel(xcoord, i, CYAN);

            prev_y_motor = y_motor;

            // complementary angle line
            drawPixel(xcoord, 230 - (int)( NewRange * (( ((float)fix2float15(complementary_angle)) + 90 ) / 180 )), RED);

            // target beam angle line
            float a_set = angle_in_deg;
            if (a_set > 90.0f)  a_set = 90.0f;
            if (a_set < -90.0f) a_set = -90.0f;

            int y_set = 230 - (int)(((a_set + 90.0f) / 180.0f) * 150.0f);
            drawPixel(xcoord, y_set, YELLOW);

            static char screentext[32];

            fillRect(30, 40, 200, 12, BLACK);
            snprintf(screentext, sizeof(screentext), "Beam angle (deg): %.2f", fix2float15(complementary_angle));
            setCursor(30, 40);
            writeString(screentext);

            fillRect(30, 50, 200, 12, BLACK);
            snprintf(screentext, sizeof(screentext), "Motor control value: %d", control);
            setCursor(30, 50);
            writeString(screentext);

            fillRect(230, 40, 200, 12, BLACK);
            snprintf(screentext, sizeof(screentext), "Kp: %.3f", Kp);
            setCursor(230, 40);
            writeString(screentext);

            fillRect(230, 50, 200, 12, BLACK);
            snprintf(screentext, sizeof(screentext), "Kd: %.3f", Kd);
            setCursor(230, 50);
            writeString(screentext);

            fillRect(230, 60, 200, 12, BLACK);
            snprintf(screentext, sizeof(screentext), "Ki: %.3f", Ki);
            setCursor(230, 60);
            writeString(screentext);

            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}



// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static int test_in ;
    static float float_in ;
    while(1) {
        sprintf(pt_serial_out_buffer, "input a command: ");

        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;

        // convert input string to number
        sscanf(pt_serial_in_buffer,"%c", &classifier) ;
        
        if (classifier=='t') {
            sprintf(pt_serial_out_buffer, "threshold: ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            if (test_in > 0) {
                threshold = test_in ;
            }
        }
        // to change target angle
        else if (classifier == 'a') {
            sprintf(pt_serial_out_buffer, "angle: ");
            serial_write;
            serial_read;

            sscanf(pt_serial_in_buffer, "%f", &float_in);
            angle_in_deg = float_in;

            if (angle_in_deg > 90.0f)  angle_in_deg = 90.0f;
            if (angle_in_deg < -90.0f) angle_in_deg = -90.0f;

        }
        // to update Kp
        else if (classifier == 'p') {
            sprintf(pt_serial_out_buffer, "kp: ");
            serial_write;
            serial_read;
;
            sscanf(pt_serial_in_buffer, "%f", &float_in);
            Kp = float_in;

        }
        // to update Kd
        else if (classifier == 'd') {
            sprintf(pt_serial_out_buffer, "kd: ");
            serial_write;
            serial_read;

            sscanf(pt_serial_in_buffer, "%f", &float_in);
            Kd = float_in;

        }
        // to update Ki
        else if (classifier == 'i') {
            sprintf(pt_serial_out_buffer, "ki: ");
            serial_write;
            serial_read;

            sscanf(pt_serial_in_buffer, "%f", &float_in);
            Ki = float_in;

        }

    }

    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Overclock
    set_sys_clock_khz(150000, true) ;

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;


    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    filtered_az = acceleration[2];
    filtered_ay = acceleration[1];

    // seed complementary filter with accelerometer estimate
    fix15 seed_accel_angle = -multfix15(float2fix15(atan2( fix2float15(filtered_ay), fix2float15(filtered_az) ) ),oneeightyoverpi);

    // Initialize complementary angle with accel
    complementary_angle = seed_accel_angle;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));

    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial);
    pt_add_thread(protothread_button);
    pt_schedule_start ;

}