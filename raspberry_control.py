import drivers
from time import sleep
import RPi.GPIO as GPIO
import serial
import time
from control.matlab import *
import math

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=.1)
FileName= "TESTE.csv"

clk = 11
dt = 13

MAX_VALUE = 1000000-1
MIN_VALUE = -1000000+1
states = ["HOME","GAIN","OBS","PAR","TEST"]
STATE = states[0]

Home_messages = ["   Controlador  ","   Observador   ","   Parametros   ","      Teste     "]
Gain_messages = ["k1:             ","k2:             ","k3:             ","k4:             "]
Obs_messages = ["l11:            ","l12:            ","l13:            ","l14:            ","l21:            ","l22:            ","l23:            ","l24:            "]
Par_messages = ["Raio da Bola:   ","Raio da Roda:   ","Massa da Bola:  ","Inercia da Roda:","Const. Torque:  ","Resis. Armadura:","Gravidade:      "]
Test_messages = ["Tempo:          ","Contagem:       "]


GPIO.setmode(GPIO.BOARD)
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

display = drivers.Lcd()
display.lcd_backlight(1)

counter_change=False
counter = 0
home_counter = 0
gain_counter = 0
obs_counter = 0
par_counter = 0
test_counter = 0
counter_counter = 1

clkLastState = GPIO.input(clk)
clkChange = False

K = [29.393,0,4.183,-0.13]
PAR = [0.026,0.124,0.045,0.00045,0.069,3.768,9.807]
A = [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
C = [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
B = [0.0,0.0,0.0,0.0]
L = [[0.537,0.012],[0.008,0.699],[4.599,-0.821],[0.582,5.581]]
L1 = [[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]]
TEMPO = 0

def round_to_n(x,n):
    if x == 0:
        return x
    else:
        return round(x, -int(math.floor(math.log10(abs(x)))) + (n - 1))
        
def format_number(x):
    x = round_to_n(x,7)
    x = "{:.7e}".format(x)
    x = x.replace('e','E')
    return x
    
def read():
    data = arduino.readline()
    print(data)
    
def enviar_cmd(cmd):
    arduino.write(bytes(cmd, 'utf-8'))
    time.sleep(0.05)

def home_page():
    global STATE
    display.lcd_clear()
    display.lcd_display_string(Home_messages[home_counter], 2)
    display.lcd_display_string("Sistema BolaRoda", 1)
    STATE = states[0]

def read_encoder():
    global counter, clkLastState, counter_change, counter_counter, clkChange
    clkState = GPIO.input(clk)
    dtState = GPIO.input(dt)
    if clkState != clkLastState:
        counter_change=True
        if dtState != clkState:
            counter = counter + 1*counter_counter
            if counter>MAX_VALUE:
                counter = MAX_VALUE
                    
        elif dtState == clkState:
            counter = counter - 1*counter_counter
            if counter<MIN_VALUE:
                counter = MIN_VALUE
        clkChange = True
        
def Obs_Calculate():
    global A, B, C, L, L1
    for i_mult in range(len(L)):
        for j_mult in range(len(C[0])):
            for k_mult in range(len(C)):
                L1[i_mult][j_mult] += L[i_mult][k_mult]*C[k_mult][j_mult]
            L1[i_mult][j_mult] = A[i_mult][j_mult] - L1[i_mult][j_mult]
    
def Model_Calculate():
    global A,B,C
    rb = PAR[0]
    rw = PAR[1]
    mb = PAR[2]
    Iw = PAR[3]
    km = PAR[4]
    ra = PAR[5]
    g = PAR[6]
    
    a=(-((2*rw*(km*km))/(ra*(7*Iw+2*(rw*rw)*mb)*(rb+rw))))
    b=((g*(5*Iw+2*(rw*rw)*mb))/((7*Iw+2*(rw*rw)*mb)*(rb+rw)))
    c=(2*rw*km)/(ra*((7*Iw+ (2*(rw*rw)*mb))*(rb+rw)))
    p=(-(7*(km*km))/(ra*(7*Iw+2*(rw*rw)*mb)))
    q=(2*g*rw*mb)/(7*Iw+2*(rw*rw)*mb)
    r=(7*km)/(ra*(7*Iw+2*(rw*rw)*mb))
    
    Asys = [[0,0,1,0],[0,0,0,1],[b,0,0,a],[q,0,0,p]]
    Bsys = [[0],[0],[c],[r]]
    Csys = [[1,0,0,0],[0,1,0,0]]
    sys = ss(Asys,Bsys,Csys,[[0],[0]])
    sysD = c2d(sys,0.02)
    Asysd = sysD.A
    Bsysd = sysD.B
    Csysd = sysD.C
    
    
    
    A = Asysd.tolist()
    C = Csysd.tolist()
    
    for row in range(4):
        B[row] = float(Bsysd[row][0])
        
def callback(channel):
    global counter_counter, counter_change, counter, K, L1, L, B, STATE, gain_counter, obs_counter, par_counter, test_counter, TEMPO, states, home_counter
    time_when_pressed = time.time()
    while(GPIO.input(18)!=GPIO.HIGH and (time.time()-time_when_pressed)<=1.0):
        pass
    total_time = time.time() - time_when_pressed
    if(STATE==states[0]and total_time>0.01):
        STATE=states[home_counter+1]
        display.lcd_clear()
        if(home_counter==0):
            counter = int(K[gain_counter]/0.001)
        elif(home_counter==1):
            if obs_counter>=4:
                counter = int(L[obs_counter-4][1]/0.001)
            else:
                counter = int(L[obs_counter][0]/0.001)
        elif(home_counter==2):
            counter = int(PAR[par_counter]/0.00005)
        elif(home_counter==3):
            counter = int(TEMPO/2)
        counter_change=True
    elif(total_time>1.0):
        display.lcd_clear()
        counter_counter=1
        if(STATE==states[1]):
            K[gain_counter] = counter*0.001
            gain_counter = gain_counter+1
            if(gain_counter>=4):
                gain_counter = 0
                enviar_cmd("K")
                for k in K:
                    enviar_cmd(format_number(k) + ";")
                read()
                STATE = states[0]
                counter = 0
                home_page()
            else:
                display.lcd_display_string(Gain_messages[gain_counter], 1)
                counter = int(K[gain_counter]/0.001)
                counter_change=True
        elif(STATE==states[2]):
            if obs_counter>=4:
                L[obs_counter-4][1] = counter*0.001
            else:
                L[obs_counter][0] = counter*0.001
            obs_counter = obs_counter+1
            if(obs_counter>=8):
                obs_counter = 0
                Obs_Calculate()
                enviar_cmd("O")
                for obs2 in L1:
                    for obs in obs2:
                        enviar_cmd(format_number(obs) + ";")
                for obs in L:
                    for obs in obs2:
                        enviar_cmd(format_number(obs)  + ";")
                for obs in B:
                    enviar_cmd(format_number(obs)  + ";")
                for n_read in range(16):
                    read()
                STATE = states[0]
                counter = 0
                home_page()
            else:
                display.lcd_display_string(Obs_messages[obs_counter], 1)
                if obs_counter>=4:
                    counter = int(L[obs_counter-4][1]/0.001)
                else:
                    counter = int(L[obs_counter][0]/0.001)
                counter_change=True
        elif(STATE==states[3]):
            PAR[par_counter] = counter*0.00005
            par_counter = par_counter + 1
            if(par_counter>=7):
                par_counter=0
                Model_Calculate()
                STATE = states[0]
                counter = 0
                home_page()
            else:
                display.lcd_display_string(Par_messages[par_counter], 1)
                counter = int(PAR[par_counter]/0.00005)
                counter_change=True
        elif(STATE==states[4] and test_counter==0):
            file = open(FileName, "w")
            TEMPO = counter*2
            test_counter=1
            enviar_cmd("T"+str(TEMPO)[:9] +";")
            read()
            STATE = states[0]
            counter = 0
            display.lcd_clear()
            display.lcd_display_string(Test_messages[test_counter], 1)
            serial_Time=0
            while(int(serial_Time)<int(TEMPO)):
                getData=str(arduino.readline())
                data=getData[2:][:-5]
                serial_Time = data.split(',', 1)[0]
                print(data)
                Percentage = 100*float(serial_Time)/(float(TEMPO))
                Percentage = str(Percentage).split('.',1)[0]
                file = open(FileName, "a")
                file.write(data + "\n")
                display.lcd_display_string((str(Percentage).zfill(3)[:3]+"%").center(16), 2)
            file.close()
            test_counter=0
            home_page()
    elif(total_time<=1.0 and STATE!=states[0]):
        counter_counter = counter_counter*10
        if(counter_counter>MAX_VALUE):
            counter_counter = 1


GPIO.add_event_detect(18, GPIO.FALLING, callback=callback,bouncetime=1000)

try:
    Model_Calculate()
    display.lcd_clear()
    display.lcd_display_string(Home_messages[0], 2)
    display.lcd_display_string("Sistema BolaRoda", 1)
    
    
    while True:
        read_encoder()
        if(STATE==states[0]):
            if(counter>1):
                counter=0
                home_counter = home_counter + 1
                if(home_counter>3):
                    home_counter=0
                display.lcd_display_string("                ",2)
                display.lcd_display_string(Home_messages[home_counter], 2)
            elif(counter<-1):
                counter=0
                home_counter = home_counter - 1
                if(home_counter<0):
                    home_counter=3
                display.lcd_display_string("                ",2)
                display.lcd_display_string(Home_messages[home_counter], 2)
        elif(STATE==states[1]):
            if(counter_change):
                counter_change=False
                display.lcd_display_string(Gain_messages[gain_counter], 1)
                number = str(counter*0.001)
                number1,number2 = number.split('.')
                if number1[0] == '-':
                    number = "-"+number1[1:].rjust(3,'0') + "." + number2[:3].ljust(3,'0')+" "
                else:
                    number = number1.rjust(3,'0') + "." + number2[:3].ljust(3,'0')
                display.lcd_display_string(number.center(16), 2)
        elif(STATE==states[2]):
            if(counter_change):
                counter_change=False
                display.lcd_display_string(Obs_messages[obs_counter], 1)
                number = str(counter*0.001)
                number1,number2 = number.split('.')
                if number1[0] == '-':
                    number = "-"+number1[1:].rjust(3,'0') + "." + number2[:3].ljust(3,'0')+" "
                else:
                    number = number1.rjust(3,'0') + "." + number2[:3].ljust(3,'0')
                display.lcd_display_string(number.center(16), 2)
        elif(STATE==states[3]):
            if(counter_change):
                counter_change=False
                if(counter<0):
                    counter=0
                display.lcd_display_string(Par_messages[par_counter], 1)
                display.lcd_display_string("                ",2)
                number = str(counter*0.00005)
                number1,number2 = number.split('.')
                number = number1.rjust(2,'0') + "." + number2[:5].ljust(5,'0')
                display.lcd_display_string(number.center(16), 2)
        elif(STATE==states[4]):
            if(counter_change and test_counter==0):
                counter_change=False
                if(counter<0):
                    counter=0
                display.lcd_display_string(Test_messages[test_counter], 1)
                display.lcd_display_string("                ",2)
                number = str(counter*0.02)
                number1,number2 = number.split('.')
                number = number1.rjust(5,'0') + "." + number2[:2].ljust(2,'0')
                display.lcd_display_string(number.center(16), 2)
        if(clkChange):
            clkLastState = GPIO.input(clk)
            clkChange = False
            
                
except KeyboardInterrupt:
    # If there is a KeyboardInterrupt (when you press ctrl+c), exit the program and cleanup
    print("Cleaning up!")
    display.lcd_clear()

