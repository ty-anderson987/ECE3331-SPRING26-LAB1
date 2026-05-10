# GPIO
PIN_LF = 5
PIN_LR = 4
PIN_RF = 3
PIN_RR = 2
PIN_ENC_R = 6
PIN_ENC_L = 7
PIN_SERVO1 = 8
PIN_SERVO2 = 9
PIN_TRIG1 = 10
PIN_ECHO1 = 11
PIN_TRIG2 = 12
PIN_ECHO2 = 13
PIN_IR1 = 14  # front
PIN_IR2 = 15  # front
PIN_IR_RX = 16
PIN_IR_LEFT   = 17  # side left
PIN_IR_RIGHT  = 18  # side right
PIN_BUZZER = 19
PIN_OUT2  = 20
PIN_OUT1  = 21
PIN_S0    = 22
PIN_S1    = 26
PIN_S2    = 27
PIN_S3    = 28

# I2C
ADDR_LEFT  = 0x41
ADDR_RIGHT = 0x40
ADDR_MPU   = 0x68

# Motor calibration
FWD_BASE_L   = 65535
FWD_BASE_R   = 65535
REV_BASE_L   = 65535
REV_BASE_R   = 65535
SLOW_AMOUNT  = 30000     # PID can adjust either wheel by up to 30k

# PID
KP             = 800
KI             = 80       # halved — was winding up to -13000 and causing right drift
KD             = 250      # more damping to prevent overshoot
INTEGRAL_CAP   = 40       # halved — limits windup magnitude
THRESHOLD      = 1
GYRO_DEADBAND  = 5.0     # °/s below this is vibration noise, ignored

# IMU
GYRO_OFFSET  = 2.68
HEADING_KP   = 1000

# Turn
TURN_SPEED   = 65535

# Sonar calibration
SONAR1_STOP_CM  = 15
SONAR2_STOP_CM  = 15
SONAR_OFFSET_CM = 4.0
SONAR_ROBOT_CM  = 20

# OC — straight driving
OVERCURRENT_MA = 600
WARN_MA        = 450

# OC — turning (higher thresholds, turns pull more current)
OVERCURRENT_TURN_MA = 900
WARN_TURN_MA        = 750

# Encoder
LEFT_TICKS_PER_REV  = 20
RIGHT_TICKS_PER_REV = 20
LEFT_CM_PER_TICK  = 20.1 / LEFT_TICKS_PER_REV
RIGHT_CM_PER_TICK = 20.1 / RIGHT_TICKS_PER_REV

# Servo
SWEEP_CENTER = 126
SWEEP_RANGE  = 60
SWEEP_MS     = 125
SWEEP_DWELL_MS = 1000
SERVO1_SPIN  = 1000
SERVO1_STOP  = 1500

# IR codes
IR_POWER      = 0xffa25d
IR_PLAY_PAUSE = 0xff02fd
IR_1          = 0xff30cf
IR_2          = 0xff18e7
IR_3          = 0xff7a85
IR_4          = 0xff10ef
IR_5          = 0xff38c7
IR_6          = 0xff5aa5
IR_7          = 0xff42bd
IR_8          = 0xff4ab5
IR_9          = 0xff52ad
IR_0          = 0xff6897
IR_LEFT       = 0xff22dd
IR_RIGHT      = 0xffc23d
IR_UP         = 0xff629d
IR_DOWN       = 0xffa857

# PWM
PWM_FREQ     = 20000