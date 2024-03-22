from machine import Pin, PWM, Timer
import time
import network
import machine
import socket
class cell:
    Northern_neighbor = -1
    Eastern_neighbor = -1
    Southern_neighbor = -1
    Western_neighbor = -1
    visited = -1
    cost = -1
    def __init__(self,visited):
        self.visited=visited

# Constants
#WHEEL_DIAMETER_CM = 4.3  # Diameter of the wheels in cm
#ENCODER_COUNTS_PER_REVOLUTION = 206.5  # Number of encoder counts per revolution
#GEAR_RATIO = 1  # Gear ratio of the motor (if any)
#DISTANCE_CM = 18  # Distance to travel in cm

# Motor right Pins
M1_PWM_PIN = 13
M1_IN1_PIN = 14
M1_IN2_PIN = 12
M1_ENCODER_PIN_A = 32
M1_ENCODER_PIN_B = 33

# Motor left Pins
M2_PWM_PIN = 25
M2_IN1_PIN = 26
M2_IN2_PIN = 27
M2_ENCODER_PIN_A = 35
M2_ENCODER_PIN_B = 34


# Global variables
total_counts_m1 = 0
total_counts_m2 = 0

def motor_setup(pwm_pin, in1_pin, in2_pin):
    pwm = PWM(Pin(pwm_pin), freq=1000, duty=0)
    in1 = Pin(in1_pin, Pin.OUT)
    in2 = Pin(in2_pin, Pin.OUT)
    return pwm, in1, in2

motor1_pwm, motor1_in1, motor1_in2 = motor_setup(M1_PWM_PIN, M1_IN1_PIN, M1_IN2_PIN)
motor2_pwm, motor2_in1, motor2_in2 = motor_setup(M2_PWM_PIN, M2_IN1_PIN, M2_IN2_PIN)

def on_encoder_change_m1(pin):
    global total_counts_m1
    total_counts_m1 += 1

 
def on_encoder_change_m2(pin):
    global total_counts_m2
    total_counts_m2 += 1
    
   
encoder1_a = Pin(M1_ENCODER_PIN_A, Pin.IN)
encoder1_a.irq(trigger=Pin.IRQ_RISING, handler=on_encoder_change_m1)
   
encoder2_a = Pin(M2_ENCODER_PIN_A, Pin.IN)
encoder2_a.irq(trigger=Pin.IRQ_RISING, handler=on_encoder_change_m2)



def set_motor_speedMotor1(pwm, in1, in2, speed):
    if speed < 0:
        in1.on()
        in2.off()
        
    elif speed > 0:
        in1.off()
        in2.on()
        
    else:
        in1.off()
        in2.off()
        
    pwm.duty(abs(speed))  
    
def set_motor_speedMotor2(pwm, in1, in2, speed):

    if speed > 0:
        in1.on()
        in2.off()
        
    elif speed < 0:
        in1.off()
        in2.on()
        
    else:
        in1.off()
        in2.off()
        
    pwm.duty(abs(speed))
     
# GPIO pin connected to the IR receiver's output
rightIR_pin = Pin(19, Pin.IN)
leftIR_pin = Pin(21, Pin.IN)
def check_rightIR_signal():
    if rightIR_pin.value() == 0:  # Active low signal
        return True
    else:
        return False
def check_leftIR_signal():
    if leftIR_pin.value() == 0:  # Active low signal
        return True
    else:
        return False
# Define the trigger and echo pins
TRIG_PIN = 5  # Change to your GPIO
ECHO_PIN = 18  # Change to your GPIO

# Set up the trigger and echo pins
trigger = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

# Function to measure distance
def measure_distance():
    
    trigger.value(1)  # Set trigger high
    time.sleep_us(10)
    trigger.value(0)  # Set trigger low again
    
    while echo.value() == 0:
        signal_off = time.ticks_us()
    while echo.value() == 1:
        signal_on = time.ticks_us()
        
    time_passed = signal_on - signal_off
    distance = (time_passed * 0.0343) / 2  # Calculate distance
    return distance
    
# Calculate parameters for movement
#wheel_circumference_cm = WHEEL_DIAMETER_CM * 3.14159
#encoder_counts_per_cm = ENCODER_COUNTS_PER_REVOLUTION / (wheel_circumference_cm * GEAR_RATIO)
DISTANCE_CM1=18
encoder_counts_per_cm1 = 12.146
# Calculate target encoder counts
target_counts1 = int(DISTANCE_CM1 * encoder_counts_per_cm1)

DISTANCE_CM2=9.9745
encoder_counts_per_cm2 = 12.146
# Calculate target encoder counts
target_counts2 = int(DISTANCE_CM2 * encoder_counts_per_cm2)

print(target_counts2)
led = machine.Pin(2, machine.Pin.OUT)
led.on()
time.sleep(1.5)
led.off()
time.sleep(1.5)
led.on()
time.sleep(1.5)
led.off()


def Forward():
    set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, 420)
    set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, 420)
    flag1 = False
    flag2 = False
    while True:
        if total_counts_m1 <= (target_counts1):
            pass
        else:
            set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, 0)
            flag1 = True
            
        if total_counts_m2 <= (target_counts1):
            pass
        else:
            set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, 0)
            flag2 = True
        if flag1 and flag2:
            break
            
def turnleft():            
    set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, 420)
    set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, -420)
    flag1 = False
    flag2 = False
    while True:
        if total_counts_m1 <= (target_counts2):
            pass
        else:
            set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, 0)
            flag1 = True
        if total_counts_m2 <= (target_counts2):
            pass
        else:
            set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, 0)        
            flag2 = True
        if flag1 and flag2:
            break
def turnright():            
    set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, -420)
    set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, 420)
    flag1 = False
    flag2 = False
    while True:
        if total_counts_m1 <= (target_counts2):
            pass
        else:
            set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, 0)
            flag1=True
        if total_counts_m2 <= (target_counts2):
            pass
        else:
            set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, 0)        
            flag2 = True
        if flag1 and flag2:
            break
Yaxis=4
Xaxis=4
Cells = [[cell(visited=0) for _ in range(Yaxis)] for _ in range(Xaxis)]
location = [0,0] #x y
Goals = [[1,3],[2,3],[1,4],[2,4]]
diriction = ['N', 'E', 'S', 'W']
forward = 'N'
for x in range(Xaxis):
    for y in range(Yaxis):
        if (x - 1) == -1 and (y - 1)==-1:
            Cells[x][y].Southern_neighbor = -3
            Cells[x][y].Western_neighbor = -3
        elif (x - 1)==-1 and (y + 1)==Yaxis:
            Cells[x][y].Northern_neighbor = -3
            Cells[x][y].Western_neighbor = -3
        elif (x + 1)==Xaxis and (y - 1)==-1:
            Cells[x][y].Eastern_neighbor = -3
            Cells[x][y].Southern_neighbor = -3
        elif (x + 1)==Xaxis and (y + 1)==Yaxis:
            Cells[x][y].Eastern_neighbor = -3
            Cells[x][y].Northern_neighbor = -3
        elif (x - 1)==-1 and ((y - 1)!=-1 and (y + 1)!=Yaxis):
            Cells[x][y].Western_neighbor = -3
        elif (x + 1)==Xaxis and ((y - 1)!=-1 and (y + 1)!=Yaxis):
            Cells[x][y].Eastern_neighbor = -3
        elif (y - 1)==-1 and ((x - 1)!=-1 and (x + 1)!=Xaxis):
            Cells[x][y].Southern_neighbor = -3
        elif (y + 1)==Yaxis and ((x - 1)!=-1 and (x + 1)!=Xaxis):
            Cells[x][y].Northern_neighbor = -3
            
while True:
    
    if Cells[location[0]][location[1]].visited == 0:
        Cells[location[0]][location[1]].visited = 1
        if (location[0] - 1) == -1 and (location[1] - 1)==-1:
            if forward == 'N':
                
                if measure_distance() <= 4:
                    
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
            elif forward == 'E':
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
            elif forward == 'S':

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
            elif forward == 'W':
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1

        elif (location[0] - 1)==-1 and (location[1] + 1)==Yaxis:

            if forward == 'N':
                Cells[location[0]][location[1]].Southern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1


            elif forward == 'W':
                Cells[location[0]][location[1]].Eastern_neighbor = -1
                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1

        elif (location[0] + 1)==Xaxis and (location[1] - 1)==-1:

            if forward == 'E':
                Cells[location[0]][location[1]].Western_neighbor = -1
                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1

            elif forward == 'S':
                Cells[location[0]][location[1]].Northern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1


        elif (location[0] + 1)==Xaxis and (location[1] + 1)==Yaxis:

            if forward == 'N':
                Cells[location[0]][location[1]].Southern_neighbor = -1
                if check_leftIR_signal():

                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1

            elif forward == 'E':
                Cells[location[0]][location[1]].Western_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1


        elif (location[0] - 1)==-1 and ((location[1] - 1)!=-1 and (location[1] + 1)!=Yaxis):

            if forward == 'N':
                Cells[location[0]][location[1]].Southern_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
            elif forward == 'S':
                Cells[location[0]][location[1]].Northern_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
            elif forward == 'W':
                Cells[location[0]][location[1]].Eastern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1
        elif (location[0] + 1)==Xaxis and ((location[1] - 1)!=-1 and (location[1] + 1)!=Yaxis):

            if forward == 'N':
                Cells[location[0]][location[1]].Southern_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1
            elif forward == 'E':
                Cells[location[0]][location[1]].Western_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
            elif forward == 'S':
                Cells[location[0]][location[1]].Northern_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1

        elif (location[1] - 1)==-1 and ((location[0] - 1)!=-1 and (location[0] + 1)!=Xaxis):

            if forward == 'E':
                Cells[location[0]][location[1]].Western_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
            elif forward == 'S':
                Cells[location[0]][location[1]].Northern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
            elif forward == 'W':
                Cells[location[0]][location[1]].Eastern_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
        elif (location[1] + 1)==Yaxis and ((location[0] - 1)!=-1 and (location[0] + 1)!=Xaxis):

            if forward == 'N':
                Cells[location[0]][location[1]].Southern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1
            elif forward == 'E':
                Cells[location[0]][location[1]].Western_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1
            elif forward == 'W':
                Cells[location[0]][location[1]].Eastern_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1
        else:
            if forward == 'N':
                Cells[location[0]][location[1]].Southern_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1
            elif forward == 'E':
                Cells[location[0]][location[1]].Western_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
            elif forward == 'S':
                Cells[location[0]][location[1]].Northern_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1

                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
            elif forward == 'W':
                Cells[location[0]][location[1]].Eastern_neighbor = -1
                if measure_distance() <= 4:
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                if check_leftIR_signal():
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1
    
    if not check_rightIR_signal():
            turnright()
            time.sleep(1)
            total_counts_m1 = 0
            total_counts_m2 = 0
            index = diriction.index(forward)
            if index == 3:  #
                forward = diriction[0]
            else:
                forward = diriction[index + 1]
    while measure_distance() <= 4:
        turnleft()
        time.sleep(1)
        total_counts_m1 = 0
        total_counts_m2 = 0
        index = diriction.index(forward)
        if index == 0:  #
            forward = diriction[3]
        else:
            forward = diriction[index - 1]
    if forward == 'N':
        location[1] += 1
    elif forward == 'E':
        location[0] += 1
    elif forward == 'S':
        location[1] -= 1
    elif forward == 'W':
        location[0] -= 1

    Forward()
    time.sleep(1)
    total_counts_m1 = 0
    total_counts_m2 = 0
    if location in Goals:
        
        exit()


