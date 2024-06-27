import cv2
from picamera2 import Picamera2
import time
import numpy as np
import RPi.GPIO as GPIO
import serial
from libcamera import controls

#Settings
ResX, ResY = 1532, 96 #Set resolution here
FPS = 1000 #Set a FPS cap here (Note that it caps the FPS)
Debug = False
Show_Fps = True
BASE_SPEED = 80
SENSITIVITY = 3

#Camera stuff
picam = Picamera2(0)
#webcam = Picamera2(1)
config = picam.create_video_configuration(main={"size": (ResX, ResY), "format": "RGB888"}, raw={"size": (1532, 864)})
picam.configure(config)
picam.set_controls({"FrameRate": FPS, "AfMode": controls.AfModeEnum.Continuous})

#GPIO stuff
PIN_RST_BUTTON = 10
PIN_OBSTACLE_BUTTON = 22
GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
GPIO.setup(PIN_RST_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_OBSTACLE_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
LEFT_MOTOR = 0
RIGHT_MOTOR = 1

#Arduino stuff
ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2) # wait for Nano to reboot

#FPS counter variables
frame_count = 0
start_time = time.time()

#Resizing Windows (for better performance)
if Debug:    
    cv2.namedWindow("Image from camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Image from camera", ResX, ResY)
    cv2.namedWindow("Blackline", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Blackline", ResX, ResY)
    cv2.namedWindow("Greensign", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Greensign", ResX, ResY)
    cv2.namedWindow("RedLine", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("RedLine", ResX, ResY)

#Stuff for the programm
kernel = np.ones((3, 3), np.uint8)
Green_ignored = False
Left_dose = False
green_points = []
new_sec = False
sec_track = time.time()
w_blk_old = 0
picam.start()

#Some functions
def send_motor_command(motor, direction, speed):
    speed = min(speed, 255)
    speed = speed // 4
    command = (motor << 7) | (direction << 6) | speed
    ser.write(bytes([command]))

def m(left, right, duration):
    left_direction = 1
    if left > 0:
        left_direction = 0
    right_direction = 1
    if right > 0:
        right_direction = 0
    send_motor_command(LEFT_MOTOR, left_direction, abs(left))
    send_motor_command(RIGHT_MOTOR, right_direction, abs(right))
    if duration > 0:
        time.sleep(duration / 1000)
        send_motor_command(LEFT_MOTOR, 1, 0)
        send_motor_command(RIGHT_MOTOR, 1, 0)

def button_rst_pressed():
    return GPIO.input(PIN_RST_BUTTON)

def button_obstacle_pressed():
    return GPIO.input(PIN_OBSTACLE_BUTTON)

#Main Loop
while True:
    
    #The Buttons
    if button_rst_pressed():
        m(0, 0, 0)
        time.sleep(3)
        while not button_rst_pressed():
            pass
        time.sleep(3)
       
    if button_obstacle_pressed():
        if Left_dose:
            m(-100, -100, 450)
            m(-255, 255, 300)
            m(160, 60, 3600)
            m(-255, 255, 360)
            m(-100, -100, 600)
            Left_dose = False

        elif Left_dose == False:
            m(-100, -100, 450)
            m(255, -255, 300)
            m(60, 160, 3500)
            m(255, -255, 360)
            m(-100, -100, 600)
            Left_dose = True
            

    Img_Cam = picam.capture_array()
    
    Blackline = cv2.inRange(Img_Cam, (0, 0, 0), (230, 80, 100))
    Greensign = cv2.inRange(Img_Cam, (0, 80, 0), (200, 255, 80))
    RedLine = cv2.inRange(Img_Cam, (0, 0, 65), (50, 50, 200))
    
    Blackline = cv2.erode(Blackline, kernel, iterations = 2)
    Blackline = cv2.dilate(Blackline, kernel, iterations = 2)

    Greensign = cv2.erode(Greensign, kernel, iterations = 2)
    Greensign = cv2.dilate(Greensign, kernel, iterations = 2)
    
    RedLine = cv2.erode(RedLine, kernel, iterations = 2)
    RedLine = cv2.dilate(RedLine, kernel, iterations = 2)
    
    contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_grn, hierarchy_grn = cv2.findContours(Greensign.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, hierarchy_red = cv2.findContours(RedLine.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    contours_blk_len = len(contours_blk)
    contours_grn_len = len(contours_grn)
    contours_red_len = len(contours_red)
    
    #Blackline
    if contours_blk_len > 0:
        x_blk, y_blk, w_blk, h_blk = cv2.boundingRect(contours_blk[0])
        centerx_blk = x_blk + (w_blk / 2)
        blackbox = cv2.minAreaRect(contours_blk[0])
        (x_min, y_min), (w_min, h_min), ang = blackbox
        x_last = x_min
        y_last = y_min
        setpoint = ResX / 2
        error = int(x_min - setpoint)

        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        cv2.drawContours(Img_Cam, [box], 0, (0, 0, 255), 3)
        cv2.line(Img_Cam, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)
        
        m(BASE_SPEED + (SENSITIVITY * error), BASE_SPEED - (SENSITIVITY * error), 0)
        
    #Red Line    
    if contours_red_len > 0:
        for i in range(contours_red_len):
            x_red, y_red, w_red, h_red = cv2.boundingRect(contours_red[i])
            cv2.rectangle(Img_Cam, (x_red, y_red), (x_red + w_red, y_red + h_red), (0, 255, 0), 2)
            
        if w_red > 150:
            m(255, 255, 150)
            m(0, 0, 0)
            print("Red detected, sleeping...")
            time.sleep(7)
            
    
    #All Turns
    if w_blk < (ResX - 10) and w_blk > 60 and not Green_ignored:
        countergreen = 0
        Green_ignored = True
        if h_blk > 70: 
            print("Boosting: ")
            m(255, 255, 150)
            Green_ignored = False
    

    if not Green_ignored:
        if contours_grn_len > 1:
            print("Turn 180 degrees")
            m(255, -255, 900)
            m(0, 0, 0)

        elif contours_grn_len > 0:
            m(255, 255, 50)
            m(0, 0, 0)
            for i in range(contours_grn_len):
                x_grn, y_grn, w_grn, h_grn = cv2.boundingRect(contours_grn[i])
                centerx_grn = x_grn + (w_grn / 2)
                green_points.append(centerx_grn)
                cv2.rectangle(Img_Cam, (x_grn, y_grn), (x_grn + w_grn, y_grn + h_grn), (255, 0, 0), 2)
                
        elif w_blk > 150:
            print("Go straight")
            m(255, 255, 500)
        
            
        if green_points:
            if all(point > centerx_blk for point in green_points):
                print("Right")
                m(255, 240, 325)
                m(255, -255, 400)
                m(255, 255, 100)
                m(0, 0, 0)
                green_points = []
                
            elif all(point < centerx_blk for point in green_points):
                print("Left")
                m(255, 240, 375)
                m(-255, 255, 350)
                m(255, 255, 100)
                m(0, 0, 0)
                green_points = []


    if new_sec:
        if w_blk_old > w_blk * 0.95 and w_blk_old < w_blk * 1.05:
            m(-255, 255, 100)
            m(255, -255, 100)
            SENSITIVITY = 4
            BASE_SPEED = 255
        else:
            w_blk_old = w_blk

    else:
        SENSITIVITY = 3
        BASE_SPEED = 80



    if Debug:
        cv2.imshow("Image from camera", Img_Cam)
        cv2.imshow("Blackline", Blackline)
        cv2.imshow("Greensign", Greensign)
        cv2.imshow("RedLine", RedLine)

    if Show_Fps:
        frame_count += 1
        if (time.time() - start_time) > 1:
            fps = frame_count / (time.time() - start_time)
            print(Img_Cam.shape, "at", round(fps), "fps")
            frame_count = 0
            start_time = time.time() 


    #Timer
    #1 sec
    if (time.time() - sec_track) > 1:
        new_sec = True
        sec_track = time.time()   
    else:
        new_sec = False
        

    if cv2.waitKey(1) == ord("q"):
        picam.close()
        cv2.destroyAllWindows()
        break