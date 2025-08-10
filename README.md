from machine import Pin, PWM, UART
from time import sleep
import ustruct

# ==== Motor Driver Pins ====
ENA = PWM(Pin(0))  # PWM for left motors
IN1 = Pin(2, Pin.OUT)
IN2 = Pin(3, Pin.OUT)

ENB = PWM(Pin(1))  # PWM for right motors
IN3 = Pin(4, Pin.OUT)
IN4 = Pin(5, Pin.OUT)

ENA.freq(1000)
ENB.freq(1000)

# ==== Sensors ====
# Line sensors
line_left = Pin(6, Pin.IN)
line_right = Pin(7, Pin.IN)

# Ultrasonic sensor
trig = Pin(8, Pin.OUT)
echo = Pin(9, Pin.IN)

# IR Receiver
ir_pin = Pin(10, Pin.IN)

# Bluetooth
uart = UART(0, baudrate=9600, tx=Pin(12), rx=Pin(13))

# ==== Helper Functions ====
def motors(left_speed, right_speed):
    # left motors
    if left_speed > 0:
        IN1.high()
        IN2.low()
    elif left_speed < 0:
        IN1.low()
        IN2.high()
    else:
        IN1.low()
        IN2.low()

    # right motors
    if right_speed > 0:
        IN3.high()
        IN4.low()
    elif right_speed < 0:
        IN3.low()
        IN4.high()
    else:
        IN3.low()
        IN4.low()

    ENA.duty_u16(int(abs(left_speed) * 65535 / 100))
    ENB.duty_u16(int(abs(right_speed) * 65535 / 100))

def stop():
    motors(0, 0)

def forward(speed=50):
    motors(speed, speed)

def backward(speed=50):
    motors(-speed, -speed)

def turn_left(speed=50):
    motors(-speed, speed)

def turn_right(speed=50):
    motors(speed, -speed)

# ==== Ultrasonic distance ====
def distance_cm():
    trig.low()
    sleep(0.002)
    trig.high()
    sleep(0.01)
    trig.low()
    while echo.value() == 0:
        pass
    t1 = time.ticks_us()
    while echo.value() == 1:
        pass
    t2 = time.ticks_us()
    return (t2 - t1) / 58.0

# ==== Main Modes ====
def ir_remote_mode():
    # NOTE: You'll need an IR library for Pico to decode remote buttons
    print("IR mode not fully implemented yet")

def bluetooth_mode():
    if uart.any():
        cmd = uart.read().decode().strip()
        if cmd == "F":
            forward(50)
        elif cmd == "B":
            backward(50)
        elif cmd == "L":
            turn_left(50)
        elif cmd == "R":
            turn_right(50)
        elif cmd == "S":
            stop()

def line_follow_mode():
    left_val = line_left.value()
    right_val = line_right.value()
    if left_val == 0 and right_val == 0:
        forward(40)
    elif left_val == 0 and right_val == 1:
        turn_left(40)
    elif left_val == 1 and right_val == 0:
        turn_right(40)
    else:
        stop()

def obstacle_avoid_mode():
    if distance_cm() < 15:
        stop()
        sleep(0.5)
        turn_left(50)
        sleep(0.5)
    else:
        forward(50)

# ==== Main Loop ====
mode = 1  # Change this to switch modes (1=IR, 2=Bluetooth, 3=Line follow, 4=Obstacle avoid)

while True:
    if mode == 1:
        ir_remote_mode()
    elif mode == 2:
        bluetooth_mode()
    elif mode == 3:
        line_follow_mode()
    elif mode == 4:
        obstacle_avoid_mode()
