# Codigo Torneo Chihuahua (CTCHI)  detector linea y apriltag (aun no sigue la linea)
#1 = adelante
#2=izquierda
#3=derecha
#4=fin
#TRUE == DERECHA
#FALSE == IZQUIERDO

import cv2
import numpy as np
import imutils
from FlyLib3.control.tello import Tello
from FlyLib3.math.pid import PID
import FlyLib3.vision.aruco as aruco
from FlyLib3.vision.apriltag import ApriltagDetector
import math
import time

dron = Tello()
dron.connect()
print(f"Batería: {dron.get_battery()}%")
dron.streamon()
dron.RESOLUTION_480P

estado_direccion = None

yaw_pid = PID(0.1, 0.000021, 0.2, setpoint=0)
#x_pid_line=PID(0.000002, 0.000021, 0.1, setpoint=0) ##jashdjasjdhas
x_pid_line=PID(0.2, 0.0000001, 0.1, setpoint=0) ##jashdjasjdhas
 
y_pid_april = PID(0.01, 0.000021, 0.1, setpoint=0)
x_pid_april = PID(0.01, 0.000021, 0.1, setpoint=0)
detector = ApriltagDetector(nthreads=4)
last_time = time.time()



lower_b = np.array([0])
upper_b = np.array([45])

angle=0
dron.send_rc_control(0,0,0,0)
dron.takeoff()
dron.move_down(20)
dron.move_forward(40)
#dron.move_down(20)



while True:
    now_time = time.time()
    frame = dron.get_frame_read().frame
    base = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
    frame = cv2.resize(frame,(640,480))
    base = cv2.resize(base,(640,480))
    
    frame = cv2.flip(frame,0)
    base = cv2.flip(base,0)
    
    x_center = frame.shape[1] // 2  ## CENTRO DE LA PANTALLA
    y_center = frame.shape[0] // 2  

    detections = detector.detect(frame, estimate_tag_pose=True)
    centre = []
    if detections:
        for detection in detections:
            offset_x = x_center - detection.center[0]
            offset_y = y_center - detection.center[1]
            tag_id = detection.tag_id
            #print("ALV LA LINEA Y AHUEVO EL APRILTAG")
            #dron.send_rc_control(-int(x_pid_april(-offset_x, now_time - last_time)),-int(y_pid_april(offset_y, now_time - last_time)),0,0)
            #print(f"valor pid y {}")
            #print(offset_y)
            print("April_tag rico🤤🤤")
            if -10 <= offset_x <= 10 and -10 <= offset_y <= 10:
                dron.send_rc_control(0,0,0,0)
                if tag_id == 1:
                    dron.move_forward(85)
                    estado_direccion=None
                if tag_id == 2:
                    dron.rotate_counter_clockwise(90)
                    dron.move_forward(85)
                    estado_direccion=False
                if tag_id == 3:
                    dron.rotate_clockwise(90)
                    dron.move_forward(85)
                    estado_direccion=True
                if tag_id == 4:
                    dron.land()                
            
    else:
        mask = cv2.inRange(frame, lower_b,upper_b)
        cv2.rectangle(mask, (0, 235), (640, 245), 0, -1)
        cv2.rectangle(mask, (0,0),(640,480), 0, 50)
        #cv2.rectangle(mask, (0, 0), (160,480), 0, -1)
        #cv2.rectangle(mask, (480, 0), (640, 480), 0, -1)
    
        cnts = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # find contours from mask
        cnts = imutils.grab_contours(cnts)
        print("ESTOY DENTRO DEL MODO LINEA, Y NO UN APRILTAG")
        for c in cnts:
            area = cv2.contourArea(c) # find how big countour is
            if area > 2000:       # only if countour is big enough, then
                M = cv2.moments(c)
                cx = int(M['m10'] / M['m00']) # calculate X position
                cy = int(M['m01'] / M['m00']) # calculate Y position

                centre.append((cx,cy))

                cv2.drawContours(base, [c], -1, (255,0,0), 3) #draw contours in blue
                cv2.circle(base, (cx, cy), 5, (255,0,0), -1)  # draw circle

        if len(centre) == 2:
            #cv2.line(frame,centre[0],centre[1],(255,0,0),1)
            cv2.line(base,centre[0],centre[1],(255,0,0),1)
            x1,y1 = centre[0]
            x2,y2 = centre[1]

            xm = (x1 + x2)//2
            error_c = xm - x_center 

            #cv2.putText(base,f"Valores PID: {-int(x_pid_line((error_c/2), now_time - last_time))}",(30,20),cv2.FONT_HERSHEY_TRIPLEX,0.8,(255,0,0),1,cv2.LINE_AA)
            #cv2.putText(base,f"Error C: {error_c}",(30,40),cv2.FONT_HERSHEY_TRIPLEX,0.8,(255,0,0),1,cv2.LINE_AA)

            
            cv2.circle(base,(xm,(y1 + y2)//2), 5, (255,0,0), -1)
            cv2.circle(base,(x_center,y_center),5,(0,0,255),-1)

            if -25 <= error_c <= 25:
                 forward_speed = 14 # Nuevo Cambio (Tejada): Solo crear la variable ya usada en la parte de abajo. Antes solo era el número, ahora tiene nombre de variable.
                 dron.send_rc_control(-int(x_pid_line((error_c/2), now_time - last_time)), forward_speed, 0,0) # Nuevo Cambio (Tejada): Aquí se aplica la variable forward_speed

            else:
                error_x  = x2 - x1
                if error_x != 0:
                    #angle = math.degrees(math.atan2((y1-y2),(x1-x2))) #cambiar setpoint a 90
                    angle = math.degrees(math.atan2((x1-x2),(y1-y2)))
                
                # Nuevo Cambio (Tejada): Nuevas variables para control de velocidad hacia adelante y el ángulo máximo
                max_speed = 10
                min_speed = 5
                max_angle = 90

                #max_speed = 15
                #min_speed = 8
                #max_angle = 90

                # Nuevo Cambio (Tejada): Cálculo de la velocidad hacia adelante basada en el ángulo
                forward_speed = int(max(min_speed, max_speed * (1 - min(abs(angle), max_angle) / max_angle)))
                print(forward_speed)

                cv2.putText(base,f"Valores PID: {int(yaw_pid(angle, now_time - last_time))}",(30,20),cv2.FONT_HERSHEY_TRIPLEX,0.8,(255,0,0),1,cv2.LINE_AA)
                cv2.putText(base,f"Angle: {angle:.2f}",(30,40),cv2.FONT_HERSHEY_TRIPLEX,0.8,(255,0,0),1,cv2.LINE_AA)
                cv2.putText(base,f"Velocity: {forward_speed:.2f}",(30,60),cv2.FONT_HERSHEY_TRIPLEX,0.8,(255,0,0),1,cv2.LINE_AA)
                
                #print(f"Error en X linea centroide {error_c}")
                #dron.send_rc_control(0,15, 0,0)
                #dron.send_rc_control(-int(x_pid(error_x, now_time - last_time)),10, 0,-int(yaw_pid(angle, now_time - last_time)))
                #dron.send_rc_control(-int(x_pid_line((error_c/2), now_time - last_time)), 10, 0,-int(yaw_pid(angle, now_time - last_time)))

                # Nuevo Cambio (Tejada): Aplicación de la variable forward_speed calculada arriba en lugar de un valor fijo
                dron.send_rc_control(-int(x_pid_line((error_c/2), now_time - last_time)), forward_speed, 0, int(yaw_pid(angle, now_time - last_time)))
        elif len(centre) == 3:
            dron.send_rc_control(0,10,0,0)

        elif len(centre) == 1:
            x1,y1 = centre[0]

            error_uno_x = x_center - x1
            error_uno_y = y_center - y1

            dron.send_rc_control(-int(x_pid_april(-error_uno_x, now_time - last_time)),-int(y_pid_april(error_uno_y, now_time - last_time)),0,0)
        else:
            dron.send_rc_control(0,0,0,0)
            if estado_direccion:
                dron.rotate_clockwise(30)
            if estado_direccion == False:
                dron.rotate_counter_clockwise(30)
        


    # print(frame.shape)    # shape do be (480, 640, 3)

    cv2.putText(base,f"Bateria: {dron.get_battery()}",(30,460),cv2.FONT_HERSHEY_TRIPLEX,0.8,(255,0,0),1,cv2.LINE_AA)
    #cv2.imshow('frame', frame)
    cv2.imshow('base', base)
    cv2.imshow('mask', mask)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

dron.land()
dron.streamoff()
cv2.destroyAllWindows()