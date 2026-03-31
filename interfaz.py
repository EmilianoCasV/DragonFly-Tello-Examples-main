from FlyLib3.control.tello import Tello
from FlyLib3.vision.apriltag import ApriltagDetector
import cv2
import math
from FlyLib3.math.pid import PID
import numpy as np
import time

drone = Tello()

drone.connect()
drone.streamon()
print(drone.get_battery())
drone.RESOLUTION_480P
tamano_pixeles_tag = 300
#drone.send_rc_control(0, 0, 20, 0) ## Controles para primer vuelo
#time.sleep(2)
#velocidad_frente = 15

deteccion = ApriltagDetector(nthreads=4)

kp = 0 ## Controles de PID disponibles
kd = 0
error_ciclo = 0

while True:
    frame_og = drone.get_frame_read().frame
    #cv2.resize(frame,(320,240))
    frame = cv2.resize(frame_og,(640,480))

    #roi = frame[180:240, 0:320] ## Roi de 0 a 320
    roi = frame[180:240, 0:640] ## Roi de 0 a 640
    #roi = frame[180:240, 40:280]

    base = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, mascara = cv2.threshold(roi_gray, 60, 255, cv2.THRESH_BINARY_INV)

    #frame_og = cv2.flip(frame_og,0) ## Giro con el espejo
    #roi = cv2.flip(roi,0)

    centro_masa = cv2.moments(mascara)
    detections = deteccion.detect(frame, estimate_tag_pose=True)

    #cv2.resize(frame,(320,240))
    #cv2.resize(base,(320,240))
    #cv2.resize(frame,(640, 480))
    #cv2.resize(base,(640, 480))

    x_centro = frame.shape[1] // 2
    y_centro = frame.shape[0] // 2

    if centro_masa["m00"] > 0:
        cx = int(centro_masa["m10"] / centro_masa["m00"])
        error_roi_x = x_centro - cx

        p = kp * error_roi_x
        d = kd * (error_roi_x - error_ciclo)
        coreccion = int(p + d)
        error_ciclo = error_roi_x

        coreccion = int(np.clip(coreccion, -100, 100))
        #drone.send_rc_control(0, velocidad_frente, 0, -coreccion)
        cv2.circle(roi, (cx, 30), 5, (255, 255, 255), 4) 
        
    else:
        print("Fuera de rango, reposicionandose")
        #drone.send_rc_control(0, 0, 0, 0)

    for detection in detections:
        tamano = math.hypot(detection.corners[1][0] - detection.corners[0][0], detection.corners[1][1] - detection.corners[0][1])
        centro_error_x = x_centro - detection.center[0]
        centro_error_y = y_centro - detection.center[1]

        cv2.line(frame_og, (x_centro, y_centro), (int(detection.center[0]), int(detection.center[1])), (255, 255, 255), 2)
        cv2.putText(frame, f"Error: X:{centro_error_x} Y:{-centro_error_y}", (30, 65), cv2.FONT_HERSHEY_PLAIN, 1.3, (255, 255, 255), 2)

        for i in range(4):
            cv2.circle(frame,tuple(detection.corners[i].astype(int)), 5, (255, 255, 255), 4)
    
    cv2.imshow('Camara',frame)
    #cv2.imshow('Mascara del proceso', mascara)
    cv2.imshow('ROI', roi)
    cv2.putText(frame_og, f"Nivel de bateria: {drone.get_battery()}", (30,40), cv2.FONT_HERSHEY_PLAIN, 1.5, (255,255,255), 2, cv2.LINE_AA)
    #cv2.circle(frame_og, (x_centro,y_centro), 5, (255,255,255), 4)
    
    if cv2.waitKey(1) & 0xff == ord('q'):
        break

#drone.land()
#drone.streamoff()
cv2.destroyAllWindows