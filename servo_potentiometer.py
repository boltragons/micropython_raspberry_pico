# @author Pedro Botelho
# @description Simple program to test MicroPython

from machine import ADC, PWM, Pin
from time import sleep_ms

POWER_LED_PIN = 2
SERVO_LED_PIN = 3
BUZZER_LED_PIN = 4
POTENTIOMETER_PIN = 27
SERVO_PIN = 0
BUZZER_PIN = 13

MIN_ADC_VALUE = 0
MAX_ADC_VALUE = 65535
MIN_DUTY_CYCLE = 1638
MAX_DUTY_CYCLE = 8192
MIN_NOTE_FREQ = 261
MAX_NOTE_FREQ = 493
SERVO_FREQ = 50
SQUARE_WAVE_DUTY_CYCLE = 32767

# Config POWER LED
power_led = Pin(POWER_LED_PIN, Pin.OUT)
power_led.high()

# Config SERVO LED
servo_led = Pin(SERVO_LED_PIN, Pin.OUT)

# Config BUZZER LED
buzzer_led = Pin(BUZZER_LED_PIN, Pin.OUT)

# Config Potentiometer ADC
potentiometer = ADC(Pin(POTENTIOMETER_PIN))

# Config Servo PWM
servo = PWM(Pin(SERVO_PIN))
servo.freq(SERVO_FREQ)
servo.duty_u16(MIN_DUTY_CYCLE)

# Config Buzzer PWM
buzzer = PWM(Pin(BUZZER_PIN))
buzzer.freq(MIN_NOTE_FREQ)
buzzer.duty_u16(0)

is_servo_enabled = False
is_buzzer_enabled = False

def enable_isr(pin):
    global is_servo_enabled
    is_servo_enabled = ~is_servo_enabled
    servo_led.toggle()

def buzzer_isr(pin):
    global is_buzzer_enabled
    is_buzzer_enabled = ~is_buzzer_enabled
    if(is_buzzer_enabled):
        buzzer.duty_u16(SQUARE_WAVE_DUTY_CYCLE)
    else:
        buzzer.duty_u16(0)
    buzzer_led.toggle()

# Config ENABLE BUTTON
enable_btn = Pin(14, Pin.IN)
enable_btn.irq(trigger=Pin.IRQ_RISING, handler=enable_isr)

# Config BUZZER BUTTON
buzzer_btn = Pin(15, Pin.IN)
buzzer_btn.irq(trigger=Pin.IRQ_RISING, handler=buzzer_isr)

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min

def setAngle(value):
    duty_cycle = map(value, MIN_ADC_VALUE, MAX_ADC_VALUE, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE)
    servo.duty_u16(int(duty_cycle))

def setNote(value):
    freq = map(value, MIN_ADC_VALUE, MAX_ADC_VALUE, MIN_NOTE_FREQ, MAX_NOTE_FREQ)
    buzzer.freq(int(freq))

while True:
    if(is_servo_enabled):
        adc_value = potentiometer.read_u16()
        setAngle(adc_value)
        if(is_buzzer_enabled):
            setNote(adc_value)
        sleep_ms(1)
