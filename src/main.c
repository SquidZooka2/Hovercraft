/*
2026-04-15
Team #5

This is the code for my final project, the autonomous hovercraft. Capable of solving a maze, we had to reduce it's speed for it's own good !
We were one of the only teams that was able to complete the maze successfully.
Bonus : We stopped under the bar for the extra 15% successfully. 

The logic is pretty straight forward with the only challenge being that the USS servo was sharing a PWM pin with our thrust fan.
Consequently I had to imitate a DC signal using delays with a PWM pin. 

Data Directional Registers, Ports, waveforms and timers were set with the help of documentation provided by each component's manufactuerer.
Code was running using the Arduino Nano on the old atmega bootloader (9600 baud).

Proudly,

Adam Benhamou
adam@benhamou.ca
*/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

#define BIT(x) (1 << x)

// ---------------- USS ----------------
#define Echo_pin   PD2
#define Echo_port  PORTD
#define Echo_DDR   DDRD
#define Input_pin  PIND

#define Trig_pin   PB3
#define Trig_port  PORTB
#define Trig_DDR   DDRB

// ---------------- PINS ----------------
#define THRUST_SERVO_PIN PB1   // P9 (Timer1)
#define THRUST_PWM_PIN   PD6   // P4 (OC0A)
#define LIFT_PWM_PIN     PD5   // P3 (OC0B)

// ---------------- CONSTANTS ----------------
#define THRESHOLD_CM 72


#define SERVO_LEFT    0
#define SERVO_CENTER  1500
#define SERVO_RIGHT   4000

#define T_SERVO_LEFT    0
#define T_SERVO_CENTER  1500
#define T_SERVO_RIGHT   4000

#define GYRO_SENSITIVITY 131.0f
#define MPU6050_ADDR 0x68
#define MPU6050_WAKEUP 0x6B
#define MPU6050_GYRO_Z_H 0x47

// ---------------- IMU GLOBALS ----------------
float gz_bias = 0;
float yaw = 0;
float target_yaw = 0;
float dt = 0.03;

// ---------------- FUNCTION DECL ----------------
void gpio_init();
void timer1_init();
void timer0_init();

void set_thrust_servo(uint16_t us);
void set_thrust(uint8_t speed);
void set_lift(uint8_t speed);

void set_uss_servo(uint16_t us);

void enable_thrust_pwm();
void disable_thrust_pwm();

void init_uss();
void trigger_uss();
long meas_echo_duration();
uint16_t read_distance();

// ---------------- MAIN ----------------
int main(void)
{
    gpio_init();
    timer1_init();
    timer0_init();
    init_uss();
    init_adc();
    twi_init();
    mpu6050_init();

    _delay_ms(500);
    calibrate_gyro_z();

    // Initial positioning (ONLY once)
    disable_thrust_pwm();
    set_uss_servo(SERVO_CENTER);
    enable_thrust_pwm();
    set_thrust_servo(T_SERVO_CENTER);

    target_yaw = 0;

    while (1)
    {
        uint16_t adc_val;
        uint8_t ir_dist;

        read_ir_distance(&adc_val, &ir_dist);

        if (ir_dist >= 10 && ir_dist <= 50)
        {
            set_thrust(0);
            set_lift(0);
            break;   // HARD STOP everything else (stops under the bar)
        }

        uint16_t front = read_distance();
        _delay_ms(300);

        if (front > THRESHOLD_CM)
        {
            uint16_t corrected = imu_correct_servo(T_SERVO_CENTER);
            set_thrust_servo(corrected);
            // Move forward with IMU correcting drift
            set_thrust(255);
            set_lift(230);
        }
        else
        {
            // STOP fans before scanning
            set_thrust(100);
            set_lift(0);
            _delay_ms(100);

            // Scan LEFT
            disable_thrust_pwm();
            set_uss_servo(SERVO_LEFT);
            enable_thrust_pwm();

            _delay_ms(300);
            uint16_t left = read_distance();

            // Scan RIGHT
            disable_thrust_pwm();
            set_uss_servo(SERVO_RIGHT);
            enable_thrust_pwm();

            // Scan Center
            _delay_ms(300);
            uint16_t right = read_distance();
            disable_thrust_pwm();
            set_uss_servo(SERVO_CENTER);
            enable_thrust_pwm();

            _delay_ms(300);
            uint16_t center = read_distance();

            // With 180 degree info of distances, we can always make the correct turn without hardcoding.
            // Some teams made an algorithm to solve the maze we were given specifically but my goal wasn't to get the best grade. My goal was to 
            // make an autonomous HC that can solve any maze.
         
            // Turn thrust servo
            if (left > right && left > center)
            {
                set_thrust_servo(T_SERVO_LEFT);
            }
            if (left < right && right > center)
            {
                set_thrust_servo(T_SERVO_RIGHT);
            }

            // Logic to make a turn
            set_thrust(255);
            set_lift(230);
            _delay_ms(1770);
            set_thrust_servo(T_SERVO_CENTER);
            _delay_ms(2000);


            // IMU north recalibration
            yaw = 0;
            target_yaw = 0;
            
            // Note that we never set our lift fan to 255 because we were going very fast and didnt want to take too much impact damage when colliding with a wall.
        }
    }
}

// ---------------- GPIO ----------------
void gpio_init()
{
    Trig_DDR |= BIT(Trig_pin);
    Echo_DDR &= ~BIT(Echo_pin);

    DDRB |= BIT(THRUST_SERVO_PIN);
    DDRD |= BIT(THRUST_PWM_PIN);
    DDRD |= BIT(LIFT_PWM_PIN);
}

// ---------------- TIMER1 (THRUST SERVO) ----------------
void timer1_init()
{
    DDRB |= (1 << PB1);

    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    ICR1 = 40000;
}

void set_thrust_servo(uint16_t us)
{
    OCR1A = us * 2;
}

// ---------------- TIMER0 (FANS) ----------------
void timer0_init()
{
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00) | (1 << WGM01);
    TCCR0B = (1 << CS01);
}

void set_thrust(uint8_t speed)
{
    OCR0A = speed;
}

void set_lift(uint8_t speed)
{
    OCR0B = speed;
}

// ---------------- PWM CONTROL ----------------
void enable_thrust_pwm()
{
    TCCR0A |= (1 << COM0A1);
}

void disable_thrust_pwm()
{
    TCCR0A &= ~(1 << COM0A1);
}

// ---------------- USS SERVO (PD6 SHARED) ----------------
void set_uss_servo(uint16_t us)
{
    for (int i = 0; i < 40; i++)
    {
        PORTD |= (1 << PD6);

        if (us == SERVO_LEFT)
            _delay_us(500);
        else if (us == SERVO_CENTER)
            _delay_us(1500);
        else if (us == SERVO_RIGHT)
            _delay_us(4000);

        PORTD &= ~(1 << PD6);
        _delay_ms(20);
    }
}

// ---------------- USS ----------------
void init_uss()
{
    Trig_port &= ~BIT(Trig_pin);
    Echo_port &= ~BIT(Echo_pin);
}
void trigger_uss()
{
    _delay_us(2);
    Trig_port |= BIT(Trig_pin);
    _delay_us(10);
    Trig_port &= ~BIT(Trig_pin);
}
long meas_echo_duration()
{
    const long timeout = 30000;
    long t = 0;

    while ((Input_pin & BIT(Echo_pin)) && t < timeout) _delay_us(1);
    while (!(Input_pin & BIT(Echo_pin)) && t < timeout) _delay_us(1);

    while ((Input_pin & BIT(Echo_pin)) && t < timeout)
    {
        t++;
        _delay_us(1);
    }

    if (t >= timeout) return -1;
    return t;
}
uint16_t read_distance()
{
    long echo_time;

    trigger_uss();
    echo_time = meas_echo_duration();

    if (echo_time <= 0) return 999;

    return echo_time / 30;
}
// ---------------- IR SENSOR ----------------
void init_adc(void){
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
    ADCSRA |= (1 << ADEN);
}
uint16_t read_adc(uint8_t channel){
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}
void read_ir_distance(uint16_t *adc_out, uint8_t *distance_out)
{
    uint16_t adc_reading = read_adc(1); // ADC1 = P8
    *adc_out = adc_reading;

    float voltage = ((float)adc_reading / 1023.0f) * 4.92f;

    if (voltage < 0.38f){
        *distance_out = 0;
        return;
    }

    float distance = 29.988f * powf(voltage, -1.173f);

    if (distance < 10.0f) distance = 10.0f;
    if (distance > 80.0f) distance = 80.0f;

    *distance_out = (uint8_t)distance;
}
// ---------------- TWI ----------------
void twi_init(void)
{
    TWSR = 0;
    TWBR = 12;
    TWCR = (1 << TWEN);
}

void twi_start(uint8_t addr)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));

    TWDR = addr;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

void twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

uint8_t twi_read_ack(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

uint8_t twi_read_nack(void)
{
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    return TWDR;
}

void twi_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
    _delay_us(5);
}

// ---------------- MPU ----------------
void mpu6050_init(void)
{
    twi_start((MPU6050_ADDR << 1) | 0);
    twi_write(MPU6050_WAKEUP);
    twi_write(0x00);
    twi_stop();
}

int16_t mpu6050_read_gyro_z(void)
{
    twi_start((MPU6050_ADDR << 1) | 0);
    twi_write(MPU6050_GYRO_Z_H);
    twi_stop();

    twi_start((MPU6050_ADDR << 1) | 1);
    uint8_t hi = twi_read_ack();
    uint8_t lo = twi_read_nack();
    twi_stop();

    return (int16_t)((hi << 8) | lo);
}

// ---------------- IMU LOGIC ----------------
void calibrate_gyro_z(void)
{
    long sum = 0;
    for (int i = 0; i < 300; i++)
    {
        sum += mpu6050_read_gyro_z();
        _delay_ms(2);
    }
    gz_bias = (float)sum / 300.0;
}

void update_yaw(void)
{
    float raw = mpu6050_read_gyro_z();
    float rate = (raw - gz_bias) / GYRO_SENSITIVITY;

    if (fabs(rate) < 1.0f) rate = 0;

    yaw += rate * dt;

    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
}

uint16_t imu_correct_servo(uint16_t base)
{
    update_yaw();

    float error = yaw - target_yaw;

    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    float Kp = 94.0f; // This variable should be changed for calibration 

    int32_t adjusted = base + (int32_t)(Kp * error);

    if (adjusted < 0) adjusted = 0;
    if (adjusted > 4000) adjusted = 4000;

    return (uint16_t)adjusted;
}

// Someone once told me the IMU was sitting on a desk climbing without knowing where it was going (drift joke)... 