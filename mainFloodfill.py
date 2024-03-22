from machine import Pin, PWM, Timer
import time
import network
import machine
from queue import Queue
class cell:
    Northern_neighbor = -1
    Eastern_neighbor = -1
    Southern_neighbor = -1
    Western_neighbor = -1
    visited = -1
    cost = -1
    def __init__(self,visited):
        self.visited=visited

def manhattan_distance(cell1, cell2):
    return abs(cell1[0] - cell2[0]) + abs(cell1[1] - cell2[1])
def get_neighbors_costs(Cells, location):
    neighborsCost=np.zeros(4, dtype=int)
    if Cells[location[0]][location[1]].Northern_neighbor != -2 and Cells[location[0]][location[1]].Northern_neighbor != -3:
        neighborsCost[0]=Cells[location[0]][location[1] + 1].cost
    elif Cells[location[0]][location[1]].Northern_neighbor == -2 or Cells[location[0]][location[1]].Northern_neighbor == -3:
        neighborsCost[0]=10000

    if Cells[location[0]][location[1]].Eastern_neighbor != -2 and Cells[location[0]][location[1]].Eastern_neighbor != -3:
        neighborsCost[1]=Cells[location[0] + 1][location[1]].cost
    elif Cells[location[0]][location[1]].Eastern_neighbor == -2 or Cells[location[0]][location[1]].Eastern_neighbor == -3:
        neighborsCost[1]=10000

    if Cells[location[0]][location[1]].Southern_neighbor != -2 and Cells[location[0]][location[1]].Southern_neighbor != -3:
        neighborsCost[2]=Cells[location[0]][location[1] - 1].cost
    elif Cells[location[0]][location[1]].Southern_neighbor == -2 or Cells[location[0]][location[1]].Southern_neighbor == -3:
        neighborsCost[2]=10000

    if Cells[location[0]][location[1]].Western_neighbor != -2 and Cells[location[0]][location[1]].Western_neighbor != -3:
        neighborsCost[3]=Cells[location[0] - 1][location[1]].cost
    elif Cells[location[0]][location[1]].Western_neighbor == -2 or Cells[location[0]][location[1]].Western_neighbor == -3:
        neighborsCost[3]=10000

    return neighborsCost

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
    global total_counts_m1 
    global total_counts_m2
    total_counts_m1 = 0
    total_counts_m2 = 0
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
    global total_counts_m1 
    global total_counts_m2
    total_counts_m1 = 0
    total_counts_m2 = 0
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
    set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, 300)
    set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, 300)
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
    set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, 300)
    set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, -300)
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
    set_motor_speedMotor1(motor1_pwm, motor1_in1, motor1_in2, -300)
    set_motor_speedMotor2(motor2_pwm, motor2_in1, motor2_in2, 300)
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

Cells = [[cell(visited=0) for _ in range(7)] for _ in range(4)]
location = [0,0] #x y
Goals = [[1,3],[2,3],[1,4],[2,4]]
diriction = ['N', 'E', 'S', 'W']
forward = 'N'
coun = 0
for x in range(4):
    for y in range(7):
        # Calculate the distance to all goal cells and take the minimum
        distances = [manhattan_distance((x, y), goal) for goal in Goals]
        Cells[x][y].cost = min(distances)
for goal in Goals:
    Cells[goal[0]][goal[1]].cost=0
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
queue = Queue()            
while True:
    
    if Cells[location[0]][location[1]].visited == 0:
        coun += 1
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
        elif (location[0] + 1)==4 and ((location[1] - 1)!=-1 and (location[1] + 1)!=7):

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

        elif (location[1] - 1)==-1 and ((location[0] - 1)!=-1 and (location[0] + 1)!=4):

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
    
    indexOfNextMove = 0
    minimeCell = Cells[location[0]][location[1]]
    neighborsCost = get_neighbors_costs(Cells, location)

    if Cells[location[0]][location[1]].cost <= neighborsCost[3] and Cells[location[0]][location[1]].cost <= neighborsCost[2] and Cells[location[0]][location[1]].cost <= neighborsCost[1] and  Cells[location[0]][location[1]].cost <= neighborsCost[0]:

        #Add current cell to queue
        locationInQu=[location[0],location[1]]
        queue.put(locationInQu)
        while not queue.empty(): 
            dequeued_item = queue.get()
            neighbors_costs = get_neighbors_costs(Cells, dequeued_item)
            minimum = neighbors_costs[0]
            for i in neighbors_costs:
                if i < minimum:
                    minimum=i

            if Cells[dequeued_item[0]][dequeued_item[1]].cost <= minimum:
                Cells[dequeued_item[0]][dequeued_item[1]].cost = minimum+1
                if Cells[dequeued_item[0]][dequeued_item[1]].Eastern_neighbor != -3 and Cells[dequeued_item[0]][dequeued_item[1]].Eastern_neighbor != -2:
                    locationInQu = [dequeued_item[0] + 1, dequeued_item[1]]
                    queue.put(locationInQu)
                if Cells[dequeued_item[0]][dequeued_item[1]].Western_neighbor != -3 and Cells[dequeued_item[0]][dequeued_item[1]].Western_neighbor != -2:
                    locationInQu = [dequeued_item[0] - 1, dequeued_item[1]]
                    queue.put(locationInQu)
                if Cells[dequeued_item[0]][dequeued_item[1]].Northern_neighbor != -3 and Cells[dequeued_item[0]][dequeued_item[1]].Northern_neighbor != -2:
                    locationInQu = [dequeued_item[0], dequeued_item[1] + 1]
                    queue.put(locationInQu)
                if Cells[dequeued_item[0]][dequeued_item[1]].Southern_neighbor != -3 and Cells[dequeued_item[0]][dequeued_item[1]].Southern_neighbor != -2:
                    locationInQu = [dequeued_item[0], dequeued_item[1] - 1]
                    queue.put(locationInQu)



    elif neighborsCost[0] != 10000:
        minimeCell = Cells[location[0]][location[1]+1]
        indexOfNextMove = 0
    elif neighborsCost[1] != 10000:
        minimeCell = Cells[location[0]+1][location[1]]
        indexOfNextMove = 1
    elif neighborsCost[2] != 10000:
        minimeCell = Cells[location[0]][location[1]-1]
        indexOfNextMove = 2
    elif neighborsCost[3] != 10000:
        minimeCell = Cells[location[0]-1][location[1]]
        indexOfNextMove = 3


    for index,i in enumerate(neighborsCost):
        if i < minimeCell.cost:
            if index == 0:
                minimeCell = Cells[location[0]][location[1]+1]
            if index == 1:
                minimeCell = Cells[location[0]+1][location[1]]
            if index == 2:
                minimeCell = Cells[location[0]][location[1]-1]
            if index == 3:
                minimeCell = Cells[location[0]-1][location[1]]
            indexOfNextMove = index

    if forword == "N" and indexOfNextMove == 1:
        turnright()
        time.sleep(1)
        index = diriction.index(forword)
        if index == 3:  #
            forword = diriction[0]
        else:
            forword = diriction[index + 1]
    elif  forword == "N" and indexOfNextMove == 3:
        turnleft()
        index = diriction.index(forword)
        if index == 0:  #
            forword = diriction[3]
        else:
            forword = diriction[index - 1]
    elif forword == "N" and indexOfNextMove == 2 :
        turnright()
        time.sleep(1)
        turnright()
        time.sleep(1)
        for k in range(2):
            index = diriction.index(forword)
            if index == 3:  #
                forword = diriction[0]
            else:
                forword = diriction[index + 1]

    ######1
    if  forword == "E" and indexOfNextMove == 2:
        turnright()
        time.sleep(1)
        index = diriction.index(forword)
        if index == 3:  #
            forword = diriction[0]
        else:
            forword = diriction[index + 1]
    elif  forword == "E" and indexOfNextMove == 0:
        turnleft()
        time.sleep(1)
        index = diriction.index(forword)
        if index == 0:  #
            forword = diriction[3]
        else:
            forword = diriction[index - 1]
    elif forword == "E" and indexOfNextMove == 3 :
        turnright()
        time.sleep(1)
        turnright()
        time.sleep(1)
        for i in range(2):
            index = diriction.index(forword)
            if index == 3:  #
                forword = diriction[0]
            else:
                forword = diriction[index + 1]
    #####2
    if  forword == "S" and indexOfNextMove == 3:
        turnright()
        time.sleep(1)
        index = diriction.index(forword)
        if index == 3:  #
            forword = diriction[0]
        else:
            forword = diriction[index + 1]
    elif  forword == "S" and indexOfNextMove == 1:
        turnleft()
        time.sleep(1)
        index = diriction.index(forword)
        if index == 0:  #
            forword = diriction[3]
        else:
            forword = diriction[index - 1]
    elif forword == "S" and indexOfNextMove == 0 :
        turnright()
        time.sleep(1)
        turnright()
        time.sleep(1)
        for i in range(2):
            index = diriction.index(forword)
            if index == 3:  #
                forword = diriction[0]
            else:
                forword = diriction[index + 1]
    #####3
    if  forword == "W" and indexOfNextMove == 0:
        turnright()
        time.sleep(1)
        index = diriction.index(forword)
        if index == 3:  #
            forword = diriction[0]
        else:
            forword = diriction[index + 1]
    elif  forword == "W" and indexOfNextMove == 2:
        turnleft()
        time.sleep(1)
        index = diriction.index(forword)
        if index == 0:  #
            forword = diriction[3]
        else:
            forword = diriction[index - 1]
    elif forword == "W" and indexOfNextMove == 1 :
        turnright()
        time.sleep(1)
        turnright()
        time.sleep(1)
        for i in range(2):
            index = diriction.index(forword)
            if index == 3:  #
                forword = diriction[0]
            else:
                forword = diriction[index + 1]
    
    Forward()
    print("Forward")
    time.sleep(1)
    if forword == 'N':
        location[1] += 1
    elif forword == 'E':
        location[0] += 1
    elif forword == 'S':
        location[1] -= 1
    elif forword == 'W':
        location[0] -= 1

    if location in Goals:
        print("Finish - :)")
        exit()
