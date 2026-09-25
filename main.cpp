#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>

// Physical constants of the robotic arm derived from Onshape CAD (in mm)
const float L1 = 100.0; 
const float L2 = 120.0; 

// Hardware PWM driver for 16-bit Timer 1 (Pins 9 and 10)
void set_servo_timer1(uint8_t channel, float angle) {
    // Input boundary sanitization
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Map 0-180 degrees to 1-2 ms pulse width (2000-4000 timer ticks)
    uint16_t ocr_val = 2000 + (uint16_t)((angle / 180.0) * 2000);
    
    if (channel == 1) OCR1A = ocr_val; // Channel A -> Digital Pin 9
    if (channel == 2) OCR1B = ocr_val; // Channel B -> Digital Pin 10
}

// Hardware PWM driver for 8-bit Timer 2 (Pin 11)
void set_servo_timer2(float angle) {
    // Input boundary sanitization
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // Map 0-180 degrees to 8-bit register values (approx. 12 to 25 ticks with 1024 prescaler)
    OCR2A = 12 + (uint8_t)((angle / 180.0) * 13);
}

// Analytical Inverse Kinematics (IK) Engine
void move_to_coords(float x, float y, float z) {
    // 1. Calculate Base angle (theta1) - rotation in X-Y plane
    float theta1 = atan2(y, x) * 180.0 / M_PI;
    
    // Projection onto the vertical plane
    float r = sqrt(x*x + y*y);
    
    // Magnitude of the vector from shoulder joint to target end-effector coordinates
    float d = sqrt(r*r + z*z);
    
    // Check if target coordinate is within the physical reach of the manipulator links
    if (d > (L1 + L2)) return; 

    // 2. Apply Law of Cosines to solve for Shoulder (theta2) and Elbow (theta3) angles
    float cos_theta3 = (d*d - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    if (cos_theta3 > 1.0) cos_theta3 = 1.0;
    if (cos_theta3 < -1.0) cos_theta3 = -1.0;
    
    float theta3 = acos(cos_theta3) * 180.0 / M_PI;
    
    float alpha = atan2(z, r);
    float beta = acos((L1*L1 + d*d - L2*L2) / (2.0 * L1 * d));
    float theta2 = (alpha + beta) * 180.0 / M_PI;

    // Update hardware registers directly with calculated angles
    set_servo_timer1(1, theta1);      // Base Joint (Pin 9)
    set_servo_timer1(2, theta2);      // Shoulder Joint (Pin 10)
    set_servo_timer2(theta3);         // Elbow Joint (Pin 11)
}

int main(void) {
    // Configure Pins 9, 10, and 11 as OUTPUT (DDRB - Data Direction Register B)
    DDRB |= (1 << DDB1) | (1 << DDB2) | (1 << DDB3);

    // Configure TIMER 1: Fast PWM Mode, Top value in ICR1 (Mode 14)
    // Non-inverting mode on Channel A and Channel B
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8
    ICR1 = 40000; // Set PWM frequency to 50 Hz (20ms period required for MG996R servos)

    // Configure TIMER 2: Fast PWM Mode (Pin 11)
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler = 1024

    while (1) {
        // Trajectory Tracking: Move end-effector between Point A and Point B
        
        // Target Point A (X=120, Y=50, Z=40)
        move_to_coords(120.0, 50.0, 40.0);
        _delay_ms(2000);
        
        // Target Point B (X=70, Y=140, Z=90)
        move_to_coords(70.0, 140.0, 90.0);
        _delay_ms(2000);
    }
    return 0;
}
