import cv2
import numpy as np
import serial
import time

def mouseRGB(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        colorsB = frame[y,x,0]
        colorsG = frame[y,x,1]
        colorsR = frame[y,x,2]
        colors = frame[y,x]
        print("Red: ",colorsR)
        print("Green: ",colorsG)
        print("Blue: ",colorsB)
        print("BRG Format: ",colors)
        print("Coordinates of pixel: X: ",x,"Y: ",y)

cap = cv2.VideoCapture(6)

# cv2.namedWindow('Image')
# cv2.setMouseCallback('Image',mouseRGB)

# Creating the kernel with numpy 
kernel2 = np.ones((4, 4), np.float32)/16
point = np.array([0,0])

ser = serial.Serial("COM3" ,
                    baudrate=9600 ,
                    bytesize = serial.EIGHTBITS,
                    parity = serial.PARITY_NONE,
                    stopbits = serial.STOPBITS_ONE,
                    timeout = 1)

time.sleep(2)

q = [70 , 90];

stri = "09900;3{};2120;1{};".format(q[0] , q[1]);
ser.write(stri.encode('ascii'))
print(ser.readline())
print(ser.readline())
print(ser.readline())
ser.flush()

while True:

    _ , frame = cap.read()
    

    newframe = frame.copy()

    cv2.line(newframe, (320 , 230), (320, 250), (255, 255, 255), thickness=4)
    cv2.line(newframe, (310 , 240), (330, 240), (255, 255, 255), thickness=4)
    

    RGB_frame = cv2.cvtColor(frame , cv2.COLOR_BGR2RGB)

    # Threshold
    lower = np.array([40, 90, 75]) 
    upper = np.array([70, 140, 120]) 

    # preparing the mask to overlay 
    mask = cv2.inRange(RGB_frame, upperb=upper, lowerb=lower)
    mask = cv2.medianBlur(mask , 3)
    mask = cv2.GaussianBlur(mask , (5,5),cv2.BORDER_DEFAULT)

    totalLabels, label_ids, values, centroid = cv2.connectedComponentsWithStats(mask, 
                                            10, 
                                            cv2.CV_32S) 
    
    values = np.delete(values[:,-1] ,[0])
    if len(values) > 0:
        
        centroid = centroid[np.argmax(values)+1]
        point = 0.01*point + 0.99*np.array([int(centroid[0]) , int(centroid[1])])

        cv2.line(newframe, (int(point[0]-10),int(point[1])), (int(point[0]+10), int(point[1])), (0, 0, 0), thickness=4)
        cv2.line(newframe, (int(point[0]) , int(point[1]-10)), (int(point[0]) , int(point[1]+10)), (0, 0, 0), thickness=4)

    cv2.imshow('mask', mask) 
    cv2.imshow("New Image" , newframe)

    # print(240 - point[1] , 320 - point[0])
    # print(q)
    # print("************************************************")
    q[0] -= (240 - point[1]) / 150;
    q[1] += (320 - point[0]) / 150;

    stri = "3{};1{};".format(int(q[0]) , int(q[1]));
    ser.write(stri.encode('ascii'))
    ser.flush()
    
    if cv2.waitKey(20) == 27:
        cap.release()
        break

ser.close()