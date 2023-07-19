import cv2
import cv2.aruco
import numpy as np

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from collections import deque

from rclpy.node import Node
from rclpy.qos import QoSProfile


from skimage.transform import ProjectiveTransform
from scipy.spatial.transform import Rotation as R

cameraMatrix = np.array([[1, 0.0, 0.0],
                         [0.0, 1.0, 0.0],
                         [0.0, 0.0, 1.0]])
distortionCoeffs = np.zeros((8), dtype=np.float32)

cameraMatrix = np.load("cameraMatrix.npy")
distortionCoeffs = np.load("distortions.npy")




class RobotController(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(Pose, 'robot_arm_goal', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        joint_states_topic = "/joint_states"
        eff_pose_topic = "/eff_pose"
        self.joint_velocity_stack = deque(maxlen=10)
        self.eff_pose_stack = deque(maxlen=10)
        
        qos = QoSProfile(depth=10)
        self.create_subscription(JointState, joint_states_topic, self.joint_state_callback)
        self.create_subscription(Pose, eff_pose_topic, self.eff_pose_callback)

    def joint_state_callback(self, data):
        self.joint_velocity_stack.append(data.velocity)

    def eff_pose_callback(self, data):
        self.eff_pose_stack.append(data)

    def get_camera_position(self):
        pose = self.eff_pose_stack.pop()
        position = [pose.position.x,
                    pose.position.y,
                    pose.position.z]
        r = R.from_quat([pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w])
        orientation = r.as_matrix()
        
        # todo: convert end factor orientation to camera orientation
        return position, orientation
    
    def goto(self, position, orientation):
        # orientation is a 3*3 matrix
        #todo: convert orientation to quaterion
        r = R.from_matrix(orientation)
        orientation = r.as_quat()
        msg = Pose()
        msg.position.x = position[0]
        msg.position.y = position[1]
        msg.position.z = position[2]
        msg.orientation.x = orientation[0]
        msg.orientation.y = orientation[1]
        msg.orientation.z = orientation[2]
        msg.orientation.w = orientation[3]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        self.i += 1
        
    def is_moving(self):
        velo = self.joint_velocity_stack.pop()
        if np.max(np.abs(velo)) > 1e-5:
            return False
        else:
            return True
        
    



class Marker():
    
    def __init__(self, corners=np.zeros((0), float), id=-1,
                 rvec=np.zeros((1, 2), float), tvec=np.zeros((1, 2), float), objPoint=np.zeros((0), float)):
        self.corners = corners
        self.id = id
        self.rvec = rvec
        self.tvec = tvec
        self.objPoint = objPoint
        self.distance = cv2.norm(tvec[0], cv2.NORM_L2)

 
        
    @property
    def center(self):
        return np.average(self.corners, axis=0)
    
    @property
    def center3d(self):
        return self.tvec



class MarkerReader():
    
    def __init__(self, markerId, markerDictionary, markerSize, cameraMatrix, distortionCoeffs):
        self.markerId = markerId
        self.cameraMatrix = cameraMatrix # needs to be calculate with a chessboard
        self.distortionCoeffs = distortionCoeffs
        
        self.markerDictionary = markerDictionary
        self.markerSize = markerSize
        self.marker = np.array([Marker(), Marker(), Marker(), Marker()])
        self.nbMarkers=0
        
        self.noDistortion=np.zeros((5), dtype=np.float32)
    
    def detectMarkers(self, img, dictionary):
        image = img.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = dictionary
        parameters = cv2.aruco.DetectorParameters_create()
        # global arucoParams
        
        (allCorners, ids, rejected) = cv2.aruco.detectMarkers(gray, dictionary, parameters=arucoParams,
                                                                    cameraMatrix=self.cameraMatrix,
                                                                    distCoeff=self.distortionCoeffs)
        
        if len(allCorners) > 0:
            self.marker = Marker()
            self.nbMarkers=0
            for i in range(0, len(ids)):
                
                (topLeft, topRight, bottomRight, bottomLeft) = allCorners[i].reshape((4, 2))
                
                rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(allCorners,
                                                                              self.markerSize,
                                                                              cameraMatrix=self.cameraMatrix,
                                                                              distCoeffs=self.distortionCoeffs)
                
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(image, allCorners) 
                
                if ids[i] == self.markerId:
                    self.nbMarkers+=1
                    self.marker = Marker(allCorners, self.markerId, rvecs[i], tvecs[i], objPoints[i])
                    cv2.aruco.drawAxis(image, self.cameraMatrix, self.distortionCoeffs, rvecs[i], tvecs[i], 51)
                    # print(self.marker.tvec)
        return image, self.marker
    
    def drawMarkers(self, img, aruco_dict_type, matrix_coefficients, distortion_coefficients):

        frame = img.copy()
        '''
        frame - Frame from the video stream
        matrix_coefficients - Intrinsic matrix of the calibrated camera
        distortion_coefficients - Distortion coefficients associated with your camera
        
        return:-
        frame - The frame with the axis drawn on it
        '''
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters_create()
        
        
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,
                                                                    parameters=parameters,
                                                                    cameraMatrix=matrix_coefficients,
                                                                    distCoeff=distortion_coefficients)

        # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                               distortion_coefficients)
                # Draw a square around the markers
                cv2.aruco.drawDetectedMarkers(frame, corners) 

                if ids[i] == self.markerId:
                    # Draw Axis
                    cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  

        return frame

        
# Chessboard
# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
ret=[]
roi=[]
rvecs=[]
tvecs=[]
captureId=0
def chessboard(img):
    global captureId
    rendered = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    foundCheckboard, corners = cv2.findChessboardCorners(gray, (7,6),
                                                        cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                        cv2.CALIB_CB_FAST_CHECK +
                                                        cv2.CALIB_CB_NORMALIZE_IMAGE)
    # cv2.imwrite('calibInput'+str(captureId)+'.png', img)
    # ret = False
    # If found, add object points, image points (after refining them)
    if foundCheckboard == True:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        
        rendered = img.copy()
        cv2.drawChessboardCorners(rendered, (7,6), corners2, foundCheckboard)
        
        # Calibration
        ret, cameraMatrix, distortionCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints,
                                                                               gray.shape[::-1],
                                                                               None, None)
        
        # Undistortion
        h, w = img.shape[:2]
        newcameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, distortionCoeffs,
                                                            (w,h), 1, (w,h))
        dst = cv2.undistort(rendered, cameraMatrix, distortionCoeffs, None, newcameraMatrix)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        if dst.shape[0] > 0 and dst.shape[1] > 0:
            rendered = dst
            cv2.imwrite('calibResult'+str(captureId)+'.png', rendered)
            captureId+=1
        
        
    # Re-projection Error 
    if foundCheckboard == True:
        mean_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i],
                                             cameraMatrix, distortionCoeffs[0])
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print( "Mean error: {}".format(mean_error/len(objpoints)) )
        #np.save("cameraMatrix.npy", cameraMatrix)
        #np.save("distortions.npy", distortionCoeffs)
    return rendered

## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

 

###############################################
##      Open CV and Numpy integration        ##
###############################################

 

import pyrealsense2 as rs
import numpy as np
import cv2
import bezier
 

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

 

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

 

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

 

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

 

# Start streaming
pipeline.start(config)

# controller
controller = RobotController()

 

try:
    while True:

 

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # Reverse image
        depth_image = depth_image[::-1,::-1]
        color_image = color_image[::-1,::-1]
        #color_image = chessboard(color_image)
        
        ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
            "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
            "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
            "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
            "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
        }
        arucoParams = cv2.aruco.DetectorParameters_create()
        
        markerId=42
        markerDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT["DICT_4X4_50"])
                
        markerReader = MarkerReader(markerId, markerDict, 51, cameraMatrix, distortionCoeffs)
        color_image, marker = markerReader.detectMarkers(color_image, markerDict)
        # color_image = markerReader.drawMarkers(color_image, ARUCO_DICT["DICT_4X4_50"], cameraMatrix, distortionCoeffs)

        ## MOVE TO X
        X=marker.tvec[0]

        controller.goto() # todo: add the position
        
        # # MOVE ROBOT by marker.tvec
        # distance = cv2.norm(marker.tvec[0], cv2.NORM_L2)
        # rotationMatrix = cv2.Rodrigues(marker.rvec)[0].reshape(3)
        # normal = rotationMatrix[2]
        # camera_nodes = np.array([[0, 0],
        #                          [0, 0 , distance / 3],
        #                          normal * distance / 3,
        #                          [marker.tvec[0]])
        # curve = bezier.Curve(nodes, degree=2)
        # curve
        # Curve (degree=2, dimension=2)
        
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

 

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

 

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

 

finally:

 

    # Stop streaming
    pipeline.stop()