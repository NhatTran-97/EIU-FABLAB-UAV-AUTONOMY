import os
import numpy as np
import cv2

ARUCO_DICT = {
    # ArUco chuan: luoi 4x4 - detect tot tu xa, it ID
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,

    # luoi 5x5
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,

    # luoi 6x6
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,

    # luoi 7x7 - nhieu ID nhat, nhung can marker to/gan moi detect duoc
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,

    # bo goc cua thu vien ArUco cu (1024 marker, luoi 5x5)

    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,

    # AprilTag (MIT): on dinh hon o khoang cach xa, decode cham hon
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

# Chi co tu OpenCV 4.7 tro len (he thong ROS2 humble dang o 4.5.4)
if hasattr(cv2.aruco, "DICT_ARUCO_MIP_36h12"):
    ARUCO_DICT["DICT_ARUCO_MIP_36h12"] = cv2.aruco.DICT_ARUCO_MIP_36h12
aruco_type = "DICT_7X7_50"
id = 4

# OpenCV >= 4.7 doi ten Dictionary_get thanh getPredefinedDictionary
if hasattr(cv2.aruco, "getPredefinedDictionary"):
    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
else:
    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])  # type: ignore


# Moi dictionary chi co so marker co dinh (DICT_4X4_50 -> ID 0..49)
max_id = arucoDict.bytesList.shape[0]
if id >= max_id:
    raise ValueError(
        "ID {} vuot gioi han cua {} (chi co ID 0..{})".format(id, aruco_type, max_id - 1)
    )

print("ArUco type '{}' with ID '{}' ".format(aruco_type, id))

tag_size = 1000
tag = np.zeros((tag_size, tag_size, 1), dtype="uint8")

# OpenCV >= 4.7 doi ten drawMarker thanh generateImageMarker
if hasattr(cv2.aruco, "generateImageMarker"):
    cv2.aruco.generateImageMarker(arucoDict, id, tag_size, tag, 1)
else:
    cv2.aruco.drawMarker(arucoDict, id, tag_size, tag, 1)  # type: ignore

# save the tag g

os.makedirs("arucoMarkers", exist_ok=True)
tag_name = "arucoMarkers/" + aruco_type + "_" + str(id) + ".png"

if not cv2.imwrite(tag_name, tag):
    raise IOError("Khong ghi duoc file: " + tag_name)
cv2.imshow("ArUCo Tag", tag)
cv2.waitKey(0)
cv2.destroyAllWindows()