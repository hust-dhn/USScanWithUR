import cv2

img = cv2.imread("camera/lc_imgs/1765801769446_0.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
corners, ids, _ = detector.detectMarkers(gray)

print("marker count:", 0 if ids is None else len(ids))
print("ids:", None if ids is None else sorted(ids.flatten().tolist()))
