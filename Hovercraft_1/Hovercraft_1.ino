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

    // Initial positioning (ONLY once)
    disable_thrust_pwm();
    set_uss_servo(SERVO_CENTER);
    enable_thrust_pwm();
    set_thrust_servo(T_SERVO_CENTER);

    while (1)
    {
        _delay_ms(300);
        uint16_t front = read_distance();

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