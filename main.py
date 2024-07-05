import cv2
from picamera2 import Picamera2
import time
import numpy as np
from gpiozero import Button
import serial
from libcamera import controls

#Settings
ResX, ResY = 1536, 96 #Set resolution here
FPS = 120 #Set a FPS cap here
Debug = False
Record = False #Still WIP
Show_Fps = True
Motors = True
BASE_SPEED = 80
SENSITIVITY = 3
Raw_Cap_Mode = 1 #Set the picam SENSOR MODE here. '1' = 1532x864; '2' = 2304x1296; 3 = 4608x2592

#Camera stuff
picam = Picamera2(0)
#webcam = Picamera2(1)
if Raw_Cap_Mode == 1:    
    config = picam.create_video_configuration(main={"size": (ResX, ResY), "format": "RGB888"}, raw={"size": (1536, 864)})
elif Raw_Cap_Mode == 2:
    config = picam.create_video_configuration(main={"size": (ResX, ResY), "format": "RGB888"}, raw={"size": (2304, 1296)})   
elif Raw_Cap_Mode == 3:
    config = picam.create_video_configuration(main={"size": (ResX, ResY), "format": "RGB888"}, raw={"size": (4608, 2592)})
picam.configure(config)
picam.set_controls({"FrameRate": FPS, "AfMode": controls.AfModeEnum.Continuous})

#GPIO stuff
button_rst = Button(10)
button_obstacle = Button(22)
LEFT_MOTOR = 0
RIGHT_MOTOR = 1

#Arduino stuff
if Motors:
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    time.sleep(2) # wait for Nano to reboot

#FPS counter variables
frame_count = 0
start_time = time.time()

#Resizing Windows (for better performance)
if Debug:    
    WX, WY = 400, ResY
    cv2.namedWindow("Image from camera", cv2.WINDOW_NORMAL)
    time.sleep(0.1)
    cv2.resizeWindow("Image from camera", WX, WY)
    time.sleep(0.1)
    cv2.namedWindow("Blackline", cv2.WINDOW_NORMAL)
    time.sleep(0.1)
    cv2.resizeWindow("Blackline", WX, WY)
    time.sleep(0.1)
    cv2.namedWindow("Greensign", cv2.WINDOW_NORMAL)
    time.sleep(0.1)
    cv2.resizeWindow("Greensign", WX, WY)
    time.sleep(0.1)
    cv2.namedWindow("RedLine", cv2.WINDOW_NORMAL)
    time.sleep(0.1)
    cv2.resizeWindow("RedLine", WX, WY)
    time.sleep(0.1)

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
def send_motor_command(motor, diRecordtion, speed):
    speed = min(speed, 255)
    speed = speed // 4
    command = (motor << 7) | (diRecordtion << 6) | speed
    if Motors:
        ser.write(bytes([command]))

def m(left, right, duration):
    left_diRecordtion = 1
    if left > 0:
        left_diRecordtion = 0
    right_diRecordtion = 1
    if right > 0:
        right_diRecordtion = 0
    send_motor_command(LEFT_MOTOR, left_diRecordtion, abs(left))
    send_motor_command(RIGHT_MOTOR, right_diRecordtion, abs(right))
    if duration > 0:
        time.sleep(duration / 1000)
        send_motor_command(LEFT_MOTOR, 1, 0)
        send_motor_command(RIGHT_MOTOR, 1, 0)
        
#Recording Stuff
if Record:
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('Debug.avi', fourcc, 120, (ResX, ResY))

#Main Loop
try:
    
    while True:
        #The Buttons
        if button_rst.is_pressed:
            m(0, 0, 0)
            time.sleep(3)
            while not button_rst.is_pressed:
                pass
            time.sleep(3)
           
        if button_obstacle.is_pressed:
            if Left_dose:
                m(-100, -100, 450)
                m(-255, 255, 300)
                m(160, 60, 3600)
                m(-255, 255, 360)
                m(-100, -100, 600)
                Left_dose = False
    
            else:
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
            cv2.line(Img_Cam, (int(x_min), ResY // 2 - ResY // 5), (int(x_min), ResY // 2 + ResY // 5), (255, 0, 0), 3)
            
            m(BASE_SPEED + (SENSITIVITY * error), BASE_SPEED - (SENSITIVITY * error), 0)
            
        #Red Line    
        if contours_red_len > 0:
            for i in range(contours_red_len):
                x_red, y_red, w_red, h_red = cv2.boundingRect(contours_red[i])
                cv2.Recordtangle(Img_Cam, (x_red, y_red), (x_red + w_red, y_red + h_red), (0, 255, 0), 2)
                
            if w_red > 150:
                m(255, 255, 150)
                m(0, 0, 0)
                if Debug:
                    print("Red detected, sleeping...")
                time.sleep(7)
                
        
        #All Turns
        if w_blk < (ResX - 10) and w_blk > 60 and not Green_ignored:
            Green_ignored = True
            if h_blk > 70: 
                if Debug:
                    print("Boosting: Turn without green")
                m(255, 255, 150)
                Green_ignored = False
        
    
        if not Green_ignored:
            if contours_grn_len > 1:
                if Debug:
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
                    cv2.Recordtangle(Img_Cam, (x_grn, y_grn), (x_grn + w_grn, y_grn + h_grn), (255, 0, 0), 2)
                    
            elif w_blk > 150:
                if Debug:
                    print("Boosting: Turn without green")
                m(255, 255, 500)
            
                
            if green_points:
                if all(point > centerx_blk for point in green_points):
                    if Debug:
                        print("Right")
                    m(255, 240, 325)
                    m(255, -255, 400)
                    m(255, 255, 100)
                    m(0, 0, 0)
                    green_points = []
                    
                elif all(point < centerx_blk for point in green_points):
                    if Debug:
                        print("Left")
                    m(255, 240, 375)
                    m(-255, 255, 350)
                    m(255, 255, 100)
                    m(0, 0, 0)
                    green_points = []
    
    
        if new_sec:
            if w_blk_old > w_blk * 0.95 and w_blk_old < w_blk * 1.05:
                if Debug:
                    print("Boosting: Stuck")
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
            
            
        if Record:
            out.write(Img_Cam)
            
    
        if Show_Fps:
            frame_count += 1
            if (time.time() - start_time) > 1:
                fps = frame_count / (time.time() - start_time)
                print(Img_Cam.shape, "at", round(fps), "fps")
                frame_count = 0
                start_time = time.time() 
    
    
        #Timer
        if (time.time() - sec_track) >= 1:
            new_sec = True
            sec_track = time.time()   
        else:
            new_sec = False
            
    
        if cv2.waitKey(1) == ord("q"):
            picam.close()
            button_rst.close()
            button_obstacle.close()
            cv2.destroyAllWindows()
            if Debug:
                out.release()
            break

except KeyboardInterrupt:
    picam.close()
    button_rst.close()
    button_obstacle.close()
    cv2.destroyAllWindows()
    if Record:
        out.release()