[1:51 pm, 03/11/2023] Ajay: import RPi.GPIO as GPIO
import cv2
import time

# Set up GPIO pins for servo control
GPIO.setmode(GPIO.BCM)
entry_servo_pin = 17
sorting_servo_pin = 18
GPIO.setup(entry_servo_pin, GPIO.OUT)
GPIO.setup(sorting_servo_pin, GPIO.OUT)
entry_servo = GPIO.PWM(entry_servo_pin, 50)
sorting_servo = GPIO.PWM(sorting_servo_pin, 50)

# Initialize the camera
cap = cv2.VideoCapture(0)

def move_servo(servo, position):
    servo.start(position)
    time.sleep(1)
    servo.stop()

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Process the image to detect tomato color (implement this part)
        # You should replace this with your color detection logic
        # In this example, we'll assume red and green objects

  …
[5:41 pm, 03/11/2023] Ajay: import RPi.GPIO as GPIO
import cv2
import time

# Set up GPIO pins for servo control
GPIO.setmode(GPIO.BCM)

# Set the GPIO pins for the entry servo and sorting servo
entry_servo_pin = 18
sorting_servo_pin = 17

# Set the PWM frequency (Hz) and duty cycle range (0-100)
pwm_frequency = 50  # 50 Hz is typical for most servo motors
duty_cycle_range = 100  # This is in percentage

# Initialize the GPIO pins for the entry servo and sorting servo
GPIO.setup(entry_servo_pin, GPIO.OUT)
GPIO.setup(sorting_servo_pin, GPIO.OUT)

# Create PWM instances for the entry servo and sorting servo
entry_servo = GPIO.PWM(entry_servo_pin, pwm_frequency)
sorting_servo = GPIO.PWM(sorting_servo_pin, pwm_frequency)

# Start PWM with a 0% duty cycle for both servos (servos at 0 degrees)
entry_servo.start(0)
sorting_servo.start(0)

# Initialize the camera
cap = cv2.VideoCapture(0)

def detect_tomato_color(frame):
    # Convert the frame to the HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define color ranges for red and green in HSV
    lower_red = (0, 100, 100)
    upper_red = (10, 255, 255)
    lower_green = (35, 100, 100)
    upper_green = (85, 255, 255)

    # Create masks for red and green regions
    mask_red = cv2.inRange(hsv_frame, lower_red, upper_red)
    mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)

    # Find contours in the masks
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours_red, contours_green

def move_servo(servo, position):
    servo.ChangeDutyCycle(2.5 + (position / 18))
    time.sleep(1)
    servo.ChangeDutyCycle(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # Detect tomato colors
        contours_red, contours_green = detect_tomato_color(frame)

        if contours_red:
            # Red tomato detected
            move_servo(entry_servo, 120)  # Open entry gate
            move_servo(sorting_servo, 10)  # Sort to the red bin
            move_servo(entry_servo, 0)  # Close entry gate
            print("Sorting red tomato")
            move_servo(entry_servo, 120)  # Move to a half-opened position1
            move_servo(sorting_servo, 90)  # Sort to the green bin
            move_servo(entry_servo, 0)  # Close entry gate
            print("Sorting green tomato")

except KeyboardInterrupt:
    pass

cap.release()
entry_servo.stop()
sorting_servo.stop()
GPIO.cleanup()
