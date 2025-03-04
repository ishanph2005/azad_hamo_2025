import cv2
import numpy as np

'''
0.37546351037169307
[[525.22292347   0.         313.92045657]
 [  0.         523.27471197 258.92116996]
 [  0.           0.           1.        ]]
[[-0.01588823 -0.28742914 -0.00226937 -0.00410108  0.57277866]]
(array([[-0.03559185],
       [-0.01584325],
       [-1.57394865]]),)
(array([[-4.34295512],
       [ 1.20215414],
       [10.4776096 ]]),)

'''


camera_matrix = np.array([[525.22292347, 0.0, 313.92045657], [0.0, 523.27471197, 258.92116996], [0.0, 0.0, 1.0]]) #focal length 300 and optical centre at centre of image
dist_coeffs = np.array([-0.01588823, -0.28742914, -0.00226937, -0.00410108,  0.57277866])
try:
    while True:
        cap = cv2.VideoCapture('/dev/video3')
        ret, frame = cap.read()
        if not ret:
            print("No frame received")
        else:
            image_from_cam = frame     
            gray = cv2.cvtColor(image_from_cam, cv2.COLOR_BGR2GRAY)
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)     
            if ids is not None:
                rot, trans, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.02, camera_matrix, dist_coeffs)
                cv2.aruco.drawDetectedMarkers(image_from_cam, corners, ids)
                cv2.drawFrameAxes(image_from_cam, camera_matrix, dist_coeffs, rot, trans, 0.05)
                print(f'Marker ID: {ids[0, 0]}')
                
                print(f'Rotation: X: {round(57.29*rot[0, 0, 0], 4)}, Y: {round(57.29*rot[0, 0, 1], 4)}, Z: {round(57.29*rot[0, 0, 2], 4)}') #in degree
                print(f'Translation: X: {round(100*trans[0, 0, 0], 4)}, Y: {round(100*trans[0, 0, 1], 4)}, Z: {round(100*trans[0, 0, 2], 4)}') #in cms
            else:
                print('No aruco markers found')
            cv2.imshow('Image with aruco', image_from_cam)
            cv2.waitKey(1)
except Exception as e:
    print(e)
    
