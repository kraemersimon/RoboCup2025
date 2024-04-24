import serial
import time
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO 

debug = False
PIN_RST_BUTTON = 10
PIN_OBSTACLE_BUTTON = 22

ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2) # wait for Nano to reboot

GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
GPIO.setup(PIN_RST_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_OBSTACLE_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

LEFT_MOTOR = 0
RIGHT_MOTOR = 1

# linefollowing paramters
BASE_SPEED = 80
SENSITIVITY = 3

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
    
camera = PiCamera()
camera.resolution = (160, 96)
camera.rotation = 0
camera.contrast = 70
camera.brightness = 60

#camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(160, 96))
start_time = time.time()
number_of_frames = 0 # frame counter for FPS
w_red = 0
w_blk = 0
w_blk_old = 0
Temp_frames = 0
counter = 0
countergreen = 0
Left_dose = True
Green_ignored = False
counterdoublegreen = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):    
    

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
    
    original_image = frame.array
    roi = original_image[0:75, 0:160]    
    kernel = np.ones((3, 3), np.uint8)
    roi = cv2.GaussianBlur(roi, ((9, 9)), 3, 3)
   
    Blackline = cv2.inRange(roi, (0, 0, 0), (230, 80, 100))
    Greensign = cv2.inRange(roi, (0, 80, 0), (200, 255, 80))
    RedLine = cv2.inRange(roi, (0, 0, 65), (50, 50, 200))
    
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
    
    if contours_blk_len > 0:
        x_blk, y_blk, w_blk, h_blk = cv2.boundingRect(contours_blk[0])
        centerx_blk = x_blk + (w_blk / 2)
        #centery_blk = y_blk + (h_blk / 2)
        if contours_blk_len == 1:
            blackbox = cv2.minAreaRect(contours_blk[0])
        else:
            canditates=[]
            off_bottom = 0    
            for con_num in range(contours_blk_len):        
                blackbox = cv2.minAreaRect(contours_blk[con_num])
                (x_min, y_min), (w_min, h_min), ang = blackbox        
                box = cv2.boxPoints(blackbox)
                (x_box,y_box) = box[0]
                if y_box > 198:        
                    off_bottom += 1
                canditates.append((y_box,con_num,x_min,y_min))        
            canditates = sorted(canditates)
            if off_bottom > 1:    
                canditates_off_bottom=[]
                for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
                    (y_highest,con_highest,x_min, y_min) = canditates[con_num]        
                    total_distance = (abs(x_min - x_last)*2 + abs(y_min - y_last)*2)*0.5
                    canditates_off_bottom.append((total_distance,con_highest))
                canditates_off_bottom = sorted(canditates_off_bottom)
                (total_distance,con_highest) = canditates_off_bottom[0]
                blackbox = cv2.minAreaRect(contours_blk[con_highest])    
            else:        
                (y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1]        
                blackbox = cv2.minAreaRect(contours_blk[con_highest])    
        (x_min, y_min), (w_min, h_min), ang = blackbox
        x_last = x_min
        y_last = y_min
        if ang < -45 :
            ang = 90 + ang
        if w_min < h_min and ang > 0:    
            ang = (90 - ang) * -1
        if w_min > h_min and ang < 0:
            ang = 90 + ang    
        setpoint = 80
        error = int(x_min - setpoint)
        ang = int(ang)
        #send_motor_command(LEFT_MOTOR, 1 if error > 0 else 0, abs(error))  # Steuerung des linken Motors basierend auf dem Fehler
        #send_motor_command(RIGHT_MOTOR, 1 if error < 0 else 0, abs(error))  # Steuerung des rechten Motors basierend auf dem Fehler

        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        cv2.drawContours(roi, [box], 0, (0, 0, 255), 3)    
        number_of_frames = number_of_frames + 1
        cv2.line(roi, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

        # set motors
        m(BASE_SPEED + (SENSITIVITY * error), BASE_SPEED - (SENSITIVITY * error), 0)

    
    if contours_red_len > 0:
        for i in range(contours_red_len):
            x_red, y_red, w_red, h_red = cv2.boundingRect(contours_red[i])
            cv2.rectangle(roi, (x_red, y_red), (x_red + w_red, y_red + h_red), (0, 255, 0), 2)
        
        if w_red > 150:
            m(255, 255, 150)
            m(0, 0, 0)
            time.sleep(7)
            
    green_points = []
    

    if w_blk < 150 and w_blk > 60 and Green_ignored == False:
        countergreen = 0
        Green_ignored = True
        if h_blk > 70: 
            m(255, 255, 150)

    
    if countergreen >= 40:
        Green_ignored = False

    if contours_grn_len > 0 and counterdoublegreen <= 0 and Green_ignored == False:
        counterdoublegreen = 20
        Green_ignored = True

    elif counterdoublegreen == 0:
        Green_ignored = False

    if Green_ignored == False:
        if contours_grn_len > 1:
            print("Turn 180 degrees")
            m(255, -255, 900)
            m(0, 0, 0)

        elif contours_grn_len > 0:
            for i in range(contours_grn_len):
                x_grn, y_grn, w_grn, h_grn = cv2.boundingRect(contours_grn[i])
                centerx_grn = x_grn + (w_grn / 2)
                #centery_grn = y_grn + (h_grn / 2)
                green_points.append(centerx_grn)
                cv2.rectangle(roi, (x_grn, y_grn), (x_grn + w_grn, y_grn + h_grn), (255, 0, 0), 2)
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
            elif all(point < centerx_blk for point in green_points):
                print("Left")
                m(255, 240, 375)
                m(-255, 255, 350)
                m(255, 255, 100)
                m(0, 0, 0)

    if debug == True:        
        cv2.imshow("orginal_image", original_image) 
        cv2.imshow("modified_image", roi) 
        cv2.imshow("black", Blackline)       
        cv2.imshow("green", Greensign)
        cv2.imshow("red", RedLine)
        print("Schwarz_Breite:", w_blk)
        #print("Rot_Breite:", w_red)
        print("Ignore Green:", Green_ignored)
        print("FPS:", str(int(number_of_frames / (time.time() - start_time))), "Error:", error)

    counter = counter + 1
    countergreen = countergreen + 1
    counterdoublegreen = counterdoublegreen - 1

    if counter == 100:
        if w_blk_old > w_blk * 0.95 and w_blk_old < w_blk * 1.05:
            m(-255, 255, 100)
            m(255, -255, 100)
            SENSITIVITY = 4
            BASE_SPEED = 255
        else:
            w_blk_old = w_blk

    elif counter == 140:
        SENSITIVITY = 3
        BASE_SPEED = 80
        counter = 0
        
        
    rawCapture.truncate(0)    
    key = cv2.waitKey(1) & 0xFF    
    if key == ord("q"):
        m(0, 0, 0)
        cv2.destroyAllWindows()
        ser.close()
        break
