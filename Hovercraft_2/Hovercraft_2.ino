#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

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
#define THRESHOLD_CM 50

#define SERVO_LEFT    0
#define SERVO_CENTER  1500
#define SERVO_RIGHT   4000

#define T_SERVO_LEFT    0
#define T_SERVO_CENTER  1500
#define T_SERVO_RIGHT   4000

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

    // Initial positioning (ONLY once)
    disable_thrust_pwm();
    set_uss_servo(SERVO_CENTER);
    enable_thrust_pwm();
    set_thrust_servo(T_SERVO_CENTER);

    while (1)
    {
        uint16_t adc_val;
        uint8_t ir_dist;
        read_ir_distance(&adc_val, &ir_dist);
        if (ir_dist >= 10 && ir_dist <= 50)
        {
            set_thrust(0);
            set_lift(0);
            break;   // HARD STOP everything else
        }

        uint16_t front = read_distance();
        _delay_ms(300);

        if (front > THRESHOLD_CM)
        {
            // Move forward (NO USS movement here anymore)
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

            _delay_ms(300);
            uint16_t right = read_distance();
            disable_thrust_pwm();
            set_uss_servo(SERVO_CENTER);
            enable_thrust_pwm();

            // Turn thrust servo
            if (left > right)
            {
                set_thrust_servo(T_SERVO_LEFT);
            }
            else
            {
                set_thrust_servo(T_SERVO_RIGHT);
            }

            // Move
            set_thrust(255);
            set_lift(230);
            _delay_ms(2500);
            set_thrust_servo(T_SERVO_CENTER);

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