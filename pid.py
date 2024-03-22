from machine import Pin, PWM, Timer
import time
import network
import machine
class cell:
    Northern_neighbor = -1
    Eastern_neighbor = -1
    Southern_neighbor = -1
    Western_neighbor = -1
    visited = -1
    cost = -1
    def __init__(self,visited):
        self.visited=visited

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

encoder1_a = Pin(M1_ENCODER_PIN_A, Pin.IN)
encoder1_b = Pin(M1_ENCODER_PIN_B, Pin.IN)

   
encoder2_a = Pin(M2_ENCODER_PIN_A, Pin.IN)
encoder2_b = Pin(M2_ENCODER_PIN_B, Pin.IN)


b1=0
b2=0

def on_encoder_change_m1F(pin):
    global total_counts_m1
    global b1
    

    b1 = encoder1_b.value()  # Read the state of ENCB pin
    if b1 == 1:
        total_counts_m1 += 1
    else:
        total_counts_m1 -= 1
    
 
def on_encoder_change_m2F(pin):
    global total_counts_m2
    global b2  

    b2 = encoder2_b.value()  # Read the state of ENCB pin
    if b2 == 0:
        total_counts_m2 += 1
    else:
        total_counts_m2 -= 1
def on_encoder_change_m1L(pin):
    global total_counts_m1
    global b1
    

    b1 = encoder1_b.value()  # Read the state of ENCB pin
    if b1 == 1:
        total_counts_m1 += 1
    else:
        total_counts_m1 -= 1
    
 
def on_encoder_change_m2L(pin):
    global total_counts_m2
    global b2  

    b2 = encoder2_b.value()  # Read the state of ENCB pin
    if b2 == 1:
        total_counts_m2 += 1
    else:
        total_counts_m2 -= 1
def on_encoder_change_m1R(pin):
    global total_counts_m1
    global b1
    

    b1 = encoder1_b.value()  # Read the state of ENCB pin
    if b1 == 0:
        total_counts_m1 += 1
    else:
        total_counts_m1 -= 1
    
 
def on_encoder_change_m2R(pin):
    global total_counts_m2
    global b2  

    b2 = encoder2_b.value()  # Read the state of ENCB pin
    if b2 == 0:
        total_counts_m2 += 1
    else:
        total_counts_m2 -= 1
   




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
        time.sleep(0.1)  # Debounce delay
        return "Yes"
    else:
        return "No"
def check_leftIR_signal():
    if leftIR_pin.value() == 0:  # Active low signal
        time.sleep(0.1)  # Debounce delay
        return "Yes"
    else:
        return "No"
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
    
DISTANCE_CM1=18
encoder_counts_per_cm1 = 12.9
# Calculate target encoder counts
target_counts1 = int(DISTANCE_CM1 * encoder_counts_per_cm1)

DISTANCE_CM2=10
encoder_counts_per_cm2 = 12.9
# Calculate target encoder counts
target_counts2 = int(DISTANCE_CM2 * encoder_counts_per_cm2)

print(target_counts1)
led = machine.Pin(2, machine.Pin.OUT)
led.on()
time.sleep(1.5)
led.off()
time.sleep(1.5)
led.on()
time.sleep(1.5)
led.off()


class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_time = 0
        self.prev_error = 0
        self.eintegral = 0

    def update(self, cur_pos, target):
        cur_time = time.ticks_us()
        deltaT = (cur_time - self.prev_time) / 1e6
        self.prev_time = cur_time

        error = target - cur_pos
        
        dedt = (error - self.prev_error) / deltaT 
        self.prev_error = error
        self.eintegral += error * deltaT

        u = self.Kp * error + self.Kd * dedt + self.Ki * self.eintegral
        pwr = abs(u)

        # Ensure pwr does not exceed 255
        if pwr > 1023:
            pwr = 1023
        
        # Determine motor direction
        
        if u < 0:
            pwr = -pwr

        return pwr

def Forward():
    global motor1_pwm
    global motor2_pwm

  
    encoder1_a.irq(trigger=Pin.IRQ_RISING, handler=on_encoder_change_m1F)   
    encoder2_a.irq(trigger=Pin.IRQ_RISING, handler=on_encoder_change_m2F)
    pid1 = PID(Kp=3.1, Ki=0, Kd=0.01)
    pid2 = PID(Kp=3.1, Ki=0, Kd=0.01)
    flag1 = False
    flag2 = False
    feedback1 = total_counts_m1  # Replace this with actual feedback reading
    feedback2 = total_counts_m2
    # Compute PID output
    pid_output1 = pid1.update(feedback1,target_counts1)
    pid_output2 = pid2.update(feedback2,target_counts1)
    set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, int(pid_output1))
    set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, int(pid_output2))
    
    while True:
        
        feedback1 = total_counts_m1  # Replace this with actual feedback reading
        feedback2 = total_counts_m2
        # Compute PID output
        pid_output1 = pid1.update(feedback1,target_counts1)
        pid_output2 = pid2.update(feedback2,target_counts1)
        
        set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, int(pid_output1))
        set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, int(pid_output2))
        
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
    print("Call")
    global inta1
    global inta2
    inta1=1
    inta2=1
    encoder1_a.irq(trigger=Pin.IRQ_RISING, handler=on_encoder_change_m1L)   
    encoder2_a.irq(trigger=Pin.IRQ_RISING, handler=on_encoder_change_m2L)
    global motor1_pwm
    global motor2_pwm
    pid1 = PID(Kp=3.1, Ki=0, Kd=0.01)
    pid2 = PID(Kp=3.1, Ki=0, Kd=0.01)
    flag1 = False
    flag2 = False
    feedback1 = total_counts_m1  # Replace this with actual feedback reading
    feedback2 = total_counts_m2
    # Compute PID output
    pid_output1 = pid1.update(feedback1,target_counts2)
    pid_output2 = pid2.update(feedback2,target_counts2)
    set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, int(pid_output1))
    set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, -int(pid_output2))
    
    while True:
        feedback1 = total_counts_m1  # Replace this with actual feedback reading
        feedback2 = total_counts_m2
        # Compute PID output
        pid_output1 = pid1.update(feedback1,target_counts2)
        pid_output2 = pid2.update(feedback2,target_counts2)
        
        set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, int(pid_output1))
        set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, -int(pid_output2))
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
    global inta1
    global inta2
    inta1=0
    inta2=0
    encoder1_a.irq(trigger=Pin.IRQ_RISING, handler=on_encoder_change_m1R)   
    encoder2_a.irq(trigger=Pin.IRQ_RISING, handler=on_encoder_change_m2R)
    global motor1_pwm
    global motor2_pwm
    pid1 = PID(Kp=3.1, Ki=0, Kd=0.01)
    pid2 = PID(Kp=3.1, Ki=0, Kd=0.01)
    flag1 = False
    flag2 = False
    feedback1 = total_counts_m1  # Replace this with actual feedback reading
    feedback2 = total_counts_m2
    # Compute PID output
    pid_output1 = pid1.update(feedback1,target_counts2)
    pid_output2 = pid2.update(feedback2,target_counts2)
    set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, -int(pid_output1))
    set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, int(pid_output2))
    
    while True:
        feedback1 = total_counts_m1  # Replace this with actual feedback reading
        feedback2 = total_counts_m2
        # Compute PID output
        pid_output1 = pid1.update(feedback1,target_counts2)
        pid_output2 = pid2.update(feedback2,target_counts2)
        
        set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, -int(pid_output1))
        set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, int(pid_output2))
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




Cells = [[cell(visited=0) for _ in range(7)] for _ in range(4)]
location = [0,0] #x y
Goals = [[1,3],[2,3],[1,4],[2,4]]
diriction = ['N', 'E', 'S', 'W']
forward = 'N'
for x in range(4):
    for y in range(7):
        if (x - 1) == -1 and (y - 1)==-1:
            Cells[x][y].Southern_neighbor = -3
            Cells[x][y].Western_neighbor = -3
        elif (x - 1)==-1 and (y + 1)==7:
            Cells[x][y].Northern_neighbor = -3
            Cells[x][y].Western_neighbor = -3
        elif (x + 1)==4 and (y - 1)==-1:
            Cells[x][y].Eastern_neighbor = -3
            Cells[x][y].Southern_neighbor = -3
        elif (x + 1)==4 and (y + 1)==7:
            Cells[x][y].Eastern_neighbor = -3
            Cells[x][y].Northern_neighbor = -3
        elif (x - 1)==-1 and ((y - 1)!=-1 and (y + 1)!=7):
            Cells[x][y].Western_neighbor = -3
        elif (x + 1)==4 and ((y - 1)!=-1 and (y + 1)!=7):
            Cells[x][y].Eastern_neighbor = -3
        elif (y - 1)==-1 and ((x - 1)!=-1 and (x + 1)!=4):
            Cells[x][y].Southern_neighbor = -3
        elif (y + 1)==7 and ((x - 1)!=-1 and (x + 1)!=4):
            Cells[x][y].Northern_neighbor = -3
            
while True:
    print("Hello")
    print(location[0],location[1])
    if Cells[location[0]][location[1]].visited == 0:
        Cells[location[0]][location[1]].visited = 1
        if (location[0] - 1) == -1 and (location[1] - 1)==-1:
            if forward == 'N':
                print("Hello",measure_distance())
                if measure_distance() <= 5:
                    
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
            elif forward == 'E':
                if measure_distance() <= 5:
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

        elif (location[0] - 1)==-1 and (location[1] + 1)==7:

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

        elif (location[0] + 1)==4 and (location[1] - 1)==-1:

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


        elif (location[0] + 1)==4 and (location[1] + 1)==7:

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


        elif (location[0] - 1)==-1 and ((location[1] - 1)!=-1 and (location[1] + 1)!=7):

            if forward == 'N':
                Cells[location[0]][location[1]].Southern_neighbor = -1
                if measure_distance() <= 5:
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
            elif forward == 'S':
                Cells[location[0]][location[1]].Northern_neighbor = -1
                if measure_distance() <= 5:
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
        elif (location[0] + 1)==4 and ((location[1] - 1)!=-1 and (location[1] + 1)!=7):

            if forward == 'N':
                Cells[location[0]][location[1]].Southern_neighbor = -1
                if measure_distance() <= 5:
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
                if measure_distance() <= 5:
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1

        elif (location[1] - 1)==-1 and ((location[0] - 1)!=-1 and (location[0] + 1)!=4):

            if forward == 'E':
                Cells[location[0]][location[1]].Western_neighbor = -1
                if measure_distance() <= 5:
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
                if measure_distance() <= 5:
                    Cells[location[0]][location[1]].Western_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Western_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Northern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Northern_neighbor = -1
        elif (location[1] + 1)==7 and ((location[0] - 1)!=-1 and (location[0] + 1)!=4):

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
                if measure_distance() <= 5:
                    Cells[location[0]][location[1]].Eastern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Eastern_neighbor = -1
                if check_rightIR_signal():
                    Cells[location[0]][location[1]].Southern_neighbor = -2
                else:
                    Cells[location[0]][location[1]].Southern_neighbor = -1
            elif forward == 'W':
                Cells[location[0]][location[1]].Eastern_neighbor = -1
                if measure_distance() <= 5:
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
                if measure_distance() <= 5:
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
                if measure_distance() <= 5:
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
                if measure_distance() <= 5:
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
                if measure_distance() <= 5:
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
            encoder1_a = Pin(M1_ENCODER_PIN_A, Pin.IN)
            encoder1_b = Pin(M1_ENCODER_PIN_B, Pin.IN)
            turnright()
            total_counts_m1 =0
            total_counts_m2 =0
            time.sleep(1)
            index = diriction.index(forward)
            if index == 3:  #
                forward = diriction[0]
            else:
                forward = diriction[index + 1]
    while measure_distance() <= 5:
        encoder1_a = Pin(M1_ENCODER_PIN_A, Pin.IN)
        encoder1_b = Pin(M1_ENCODER_PIN_B, Pin.IN)
        turnleft()
        total_counts_m1 =0
        total_counts_m2 =0
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
    encoder1_a = Pin(M1_ENCODER_PIN_A, Pin.IN)
    encoder1_b = Pin(M1_ENCODER_PIN_B, Pin.IN)
    Forward()
    total_counts_m1 =0
    total_counts_m2 =0
    time.sleep(1)
    if location in Goals:
        exit()




