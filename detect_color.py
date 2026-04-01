import cv2
import numpy as np
from FlyLib3.control.tello import Tello
from FlyLib3.math.pid import PID
import time

drone = Tello()

drone.connect()
drone.streamon()
##drone.RESOLUTION_480P
#drone.takeoff()
#time.sleep(2)

print(drone.get_battery())


while True:
    frame = drone.get_frame_read().frame
    frame = cv2.resize(frame, (640,480))
    #color = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    x_centro = frame.shape[1] // 2
    y_centro = frame.shape[0] // 2

    ## --------- Rango de azul -----------#
    bajo_azul = np.array([100, 100, 20], dtype=np.uint8)
    alto_azul = np.array([130, 255, 255], dtype=np.uint8)
    mask_azul = cv2.inRange(hsv, bajo_azul, alto_azul)

    #------------Rango de rojo --------------#
    bajo_rojo1 = np.array([0, 100, 20])
    alto_rojo1 = np.array([10, 255, 255])
    bajo_rojo2 = np.array([170, 100, 20])
    alto_rojo2 = np.array([180, 255, 255])

    mask_rojo_1 = cv2.inRange(hsv, bajo_rojo1, alto_rojo1)
    mask_rojo_2 = cv2.inRange(hsv, bajo_rojo2, alto_rojo2)
    mask_rojo = cv2.add(mask_rojo_1, mask_rojo_2)

    mascara_final = cv2.bitwise_or(mask_azul, mask_rojo)
    mask = cv2.bitwise_and(frame, frame, mask=mascara_final)

    umbral = 500

    pixel_azul = cv2.countNonZero(mask_azul)
    pixel_rojo = cv2.countNonZero(mask_rojo)

    color_detectado = ""

    if pixel_azul > umbral and pixel_azul > pixel_rojo:
        color_detectado = "Azul"

    elif pixel_rojo > umbral and pixel_rojo > pixel_azul:
        color_detectado = "Rojo"

    cv2.putText(frame, f"Bateria: {drone.get_battery()}", (30,40), cv2.FONT_HERSHEY_PLAIN, 1.5, (255 ,255 ,255), 2, cv2.LINE_AA)
    cv2.putText(frame, f"Color: {color_detectado}", (30,60), cv2.FONT_HERSHEY_PLAIN, 1.5, (255 ,255 ,255), 2, cv2.LINE_AA)

    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)

    if cv2.waitKey(1) & 0xff == ord('q'):
        break

#drone.land()
cv2.destroyAllWindows