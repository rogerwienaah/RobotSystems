import sys
# sys.path.append(r'/home/kaanb/RobotSystems/lib')
import atexit
import time
import numpy
import numpy as np
import cv2
import math
try:
    from picamera2 import PiCamera2
    from picamera.array import PiRGBArray

except ImportError:
    print("No Picamera available ")
    exit()

from picarx_improved import Picarx

_SHOW_IMAGE = False


class CameraBasedFollower(Picarx):
    def __init__(self):
        super().__init__()
        self.camera = PiCamera2()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 24
        self.camera.start_preview()
        self.stream = PiRGBArray(self.camera, size=self.camera.resolution)
        time.sleep(1)

    def sensor(self):
        self.camera.capture(self.stream, format="bgr")

        return self.stream.array

    def interpreter(self):
        frame = self.sensor()
        lane_lines, frame = detect_lane(frame)
        degree = compute_steering_angle(frame, lane_lines)
        return degree

    def controller(self, scaling_factor=0.25):
        turn = -1 * scaling_factor * self.interpreter()
        self.set_dir_servo_angle(int(turn))
        time.sleep(0.01)
        return turn
    


def detect_lane(frame):

    edges = detect_edges(frame)
    show_image('edges', edges)
    cropped_edges = region_of_interest(edges)
    show_image('edges cropped', cropped_edges)
    line_segments = detect_line_segments(cropped_edges)
    line_segment_image = display_lines(frame, line_segments)
    show_image("line segments", line_segment_image)
    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    show_image("lane lines", lane_lines_image)

    return lane_lines, lane_lines_image

def detect_edges(frame):

    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    show_image("hsv", hsv)
    lower_blue = np.array([30, 40, 0])
    upper_blue = np.array([150, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    show_image("blue mask", mask)
    # detect edges
    edges = cv2.Canny(mask, 200, 400)

    return edges

def region_of_interest(canny):

    height, width = canny.shape
    mask = np.zeros_like(canny)
    # only focus bottom half of the screen
    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    show_image("mask", mask)
    masked_image = cv2.bitwise_and(canny, mask)

    return masked_image

def detect_line_segments(cropped_edges):

    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                    maxLineGap=4)

    return line_segments

def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    height, width, _ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary  # right lane line segment should be on left 2/3 of the screen
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))
    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines


def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):

    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    return line_image

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):

    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def length_of_line_segment(line):

    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def show_image(title, frame, show=_SHOW_IMAGE):
    if show:
        cv2.imshow(title, frame)

def make_points(frame, line):

    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))

    return [[x1, y1, x2, y2]]

def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02  # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg  # this is the steering angle needed by picar front wheel

    return steering_angle

if __name__ == "__main__":

    car = CameraBasedFollower()
    atexit.register(car.stop)
    while True:
        angle = car.controller()
        print(angle)
        car.forward(30)
        time.sleep(0.05)
        cv2.destroyAllWindows()
        car.stream = PiRGBArray(car.camera, size=car.camera.resolution)