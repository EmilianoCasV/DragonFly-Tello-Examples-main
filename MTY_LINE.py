# Codigo Torneo MTY detector linea y color

## Cosas para calibrar dia competencia :D
# (Ya quedó)- Calibracion de colores (rojo,azul y negro) -- esto lo pueden checar en el codigo del color, y luego hacen cambios aqui
# NOOOOOOO MOVVVEEEER EEEEELLL PIIIIIIIDDDDDDD Ya esta demaciado perfecto :D - atten: EMILIANO
# (Ya quedó)- Altura del muro
# (Ya quedó)- Distancia del primer cuadro del color al siguiente tramo de linea
# (Ya quedó)- Angulo de rotacion de las lineas de V
#- Ver la cantidad de pixeles
#- Revisar la ultima vuelta del cuadro para ver si alcanza a detectar la linea
#- GANAR EL FUCKING PRIMER LUGAR >:D

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

state_color = False
state_color_two = False


## -----------------Configuraciones color-----------------##
    ## --------- Rango de azul -----------#
bajo_azul = np.array([100, 100, 20], dtype=np.uint8)
alto_azul = np.array([130, 255, 255], dtype=np.uint8)

    #------------Rango de rojo --------------#
bajo_rojo1 = np.array([0, 100, 20])
alto_rojo1 = np.array([10, 255, 255])
bajo_rojo2 = np.array([170, 100, 20])
alto_rojo2 = np.array([180, 255, 255])

umbral = 500
color_detectado = ""
##---------------------------------------------------------##


##------------------PID Controller---------------##
yaw_pid = PID(1.3, 0.000021, 1, setpoint=0)
#x_pid_line=PID(0.000002, 0.000021, 0.1, setpoint=0) ##jashdjasjdhas
x_pid_line=PID(0.2, 0.0000001, 0.1, setpoint=0) ##jashdjasjdhas
 
y_pid_april = PID(0.01, 0.000021, 0.1, setpoint=0)
x_pid_april = PID(0.01, 0.000021, 0.1, setpoint=0)

## Esta chistoso por que el x y pid april son los que usamos para centrarnos con la linea jajaja
#---------------------------------------------------##

last_time = time.time()
state = 0


lower_b = np.array([0])
upper_b = np.array([70])

angle=0
dron.send_rc_control(0,0,0,0)
dron.takeoff()
dron.move_forward(30)
dron.streamon()
dron.RESOLUTION_480P
dron.move_down(50)


while True:
    now_time = time.time()
    
    frame = dron.get_frame_read().frame
    frame = cv2.resize(frame,(640,480))
    frame = cv2.flip(frame,0)


    base = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


    #------------Mascaras color-------------------#
    mask_azul = cv2.inRange(hsv, bajo_azul, alto_azul)
    mask_rojo_1 = cv2.inRange(hsv, bajo_rojo1, alto_rojo1)
    mask_rojo_2 = cv2.inRange(hsv, bajo_rojo2, alto_rojo2)
    mask_rojo = cv2.add(mask_rojo_1, mask_rojo_2)
    mascara_final = cv2.bitwise_or(mask_azul, mask_rojo)
    mask_color = cv2.bitwise_and(frame, frame, mask=mascara_final)


    
    pixel_azul = cv2.countNonZero(mask_azul)
    pixel_rojo = cv2.countNonZero(mask_rojo)

    
    x_center = frame.shape[1] // 2  ## CENTRO DE LA PANTALLA
    y_center = frame.shape[0] // 2  
    
    centre = []
    mask = cv2.inRange(frame, lower_b,upper_b)
    cv2.rectangle(mask, (0, 235), (640, 245), 0, -1)
    cv2.rectangle(mask, (0,0),(640,720), 0, 50)
#    mask = cv2.subtract(mask, mascara_final) # Prueba para eliminar los colores rojo y azul de la mascara de la linea

    cnts = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # find contours from mask
    cnts = imutils.grab_contours(cnts)
    print("ESTOY DENTRO DEL MODO LINEA")
    
    if pixel_azul > umbral and pixel_azul > pixel_rojo:
        color_detectado = "Azul"
    elif pixel_rojo > umbral and pixel_rojo > pixel_azul:
        color_detectado = "Rojo"
    else:
        color_detectado = ""

    

    if state_color==False:
        if (pixel_azul or pixel_rojo) > 10000:
            if (color_detectado == "Rojo") and (-10<angle<10):
                dron.move_forward(100)
                dron.rotate_clockwise(55)
                state_color = True
            if (color_detectado == "Azul") and (-10<angle<10):
                dron.move_forward(100)
                dron.rotate_counter_clockwise(55)
                state_color = True
    elif (state_color == True) and (state_color_two==False):
        if (pixel_azul or pixel_rojo) > 20000:
            if (color_detectado == "Rojo") and (-10<angle<10):
                dron.rotate_counter_clockwise(90)
                dron.move_forward(60)
                state_color_two = True
            if (color_detectado == "Azul") and (-10<angle<10):
                dron.rotate_clockwise(45)
                dron.move_forward(60)
                state_color_two = True
                


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
        cv2.line(base,centre[0],centre[1],(255,0,0),1)
        x1,y1 = centre[0]
        x2,y2 = centre[1]
        xm = (x1 + x2)//2
        error_c = xm - x_center 

         
        cv2.circle(base,(xm,(y1 + y2)//2), 5, (255,0,0), -1)
        cv2.circle(base,(x_center,y_center),5,(0,0,255),-1)

        if -15 <= error_c <= 15:
            forward_speed = 10  #14 Nuevo Cambio (Tejada): Solo crear la variable ya usada en la parte de abajo. Antes solo era el número, ahora tiene nombre de variable.
            dron.send_rc_control(-int(x_pid_line((error_c/2), now_time - last_time)), forward_speed, 0,0) # Nuevo Cambio (Tejada): Aquí se aplica la variable forward_speed

        else:
            error_x  = x2 - x1
            if error_x != 0:
                angle = math.degrees(math.atan2((x1-x2),(y1-y2)))
             
              # Nuevo Cambio (Tejada): Nuevas variables para control de velocidad hacia adelante y el ángulo máximo
                max_speed = 10
                min_speed = 1
                max_angle = 90


                # Nuevo Cambio (Tejada): Cálculo de la velocidad hacia adelante basada en el ángulo
                forward_speed = int(max(min_speed, max_speed * (1 - min(abs(angle), max_angle) / max_angle)))
                #print(forward_speed)

                cv2.putText(base,f"Valores PID: {int(yaw_pid(angle, now_time - last_time))}",(30,20),cv2.FONT_HERSHEY_TRIPLEX,0.8,(255,0,0),1,cv2.LINE_AA)
                cv2.putText(base,f"Angle: {angle:.2f}",(30,40),cv2.FONT_HERSHEY_TRIPLEX,0.8,(255,0,0),1,cv2.LINE_AA)
                cv2.putText(base,f"Velocity: {forward_speed:.2f}",(30,60),cv2.FONT_HERSHEY_TRIPLEX,0.8,(255,0,0),1,cv2.LINE_AA)

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
        None
    
    # print(frame.shape)    # shape do be (480, 640, 3)
    cv2.putText(base,f"Bateria: {dron.get_battery()}",(30,460),cv2.FONT_HERSHEY_TRIPLEX,0.8,(255,0,0),1,cv2.LINE_AA)
    cv2.putText(base, f"Color: {color_detectado}", (30,80), cv2.FONT_HERSHEY_PLAIN, 1.5, (255 ,255 ,255), 2, cv2.LINE_AA)
    cv2.putText(base, f"No.Pixeles: {pixel_azul}", (30,100), cv2.FONT_HERSHEY_PLAIN, 1.5, (255 ,255 ,255), 2, cv2.LINE_AA)


    #cv2.imshow('frame', frame)
    cv2.imshow('base', base)
    cv2.imshow('mask', mask)
    

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

dron.land()
dron.streamoff()
cv2.destroyAllWindows()