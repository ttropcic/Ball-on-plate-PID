import sys
import cv2
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import Qt
from PyQt5 import QtGui
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QDialog, QLabel, QColorDialog, QWidget
from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSlot
import imutils
import serial
import time



class GUI(QDialog):
    def __init__(self):
        super().__init__()
        loadUi('GUI_design_3.ui', self)


        # Color Buttons
        self.button_red.clicked.connect(lambda: self.change_slider_value("red"))
        self.button_orange.clicked.connect(lambda: self.change_slider_value("orange"))
        self.button_black.clicked.connect(lambda: self.change_slider_value("black"))
        # Other Buttons
        self.button_start_tracking.clicked.connect(self.start_tracking)
        self.button_start_motors.clicked.connect(self.start_motors)
        self.button_show_thresh.clicked.connect(self.show_thresh)
        self.button_reset_pid.clicked.connect(self.reset_spinboxes)
        # Color Sliders
        self.h_min_slider.valueChanged.connect(self.update_color)
        self.s_min_slider.valueChanged.connect(self.update_color)
        self.v_min_slider.valueChanged.connect(self.update_color)
        self.h_max_slider.valueChanged.connect(self.update_color)
        self.s_max_slider.valueChanged.connect(self.update_color)
        self.v_max_slider.valueChanged.connect(self.update_color)
        # PID Spinboxes
        self.p_box.valueChanged.connect(self.update_spinbox)
        self.i_box.valueChanged.connect(self.update_spinbox)
        self.d_box.valueChanged.connect(self.update_spinbox)
        self.max_int_box.valueChanged.connect(self.update_spinbox)
        # Servo Calibration Spinboxes
        self.x_box.valueChanged.connect(self.update_start_angle)
        self.y_box.valueChanged.connect(self.update_start_angle)

        # Cam label
        self.cam_label.mousePressEvent = self.mouse_event

        # camera settings
        self.cam_width = 640
        self.cam_height = 480

        # PID
        self.default_P = 0.033
        self.default_D = 0.023
        self.default_I = 0
        self.kP, self.kD, self.kI = self.default_P, self.default_D, self.default_I
        self.last_time = 0
        self.default_setpoint_x = self.cam_width / 2
        self.default_setpoint_y = self.cam_height / 2
        self.setPointX, self.setPointY = self.default_setpoint_x, self.default_setpoint_y
        self.lastErrorX, self.lastErrorY = 0, 0
        self.errorSumX, self.errorSumY = 0, 0
        self.max_error_sum = 0
        self.zero_x = 31
        self.zero_y = 35

        # Colors
        self.my_colors = {"orange": ([0, 77, 115], [61, 153, 255]),
                          "red": ([121, 157, 86], [243, 255, 255]),
                          "black": ([0, 0, 0], [25, 25, 25])}
        self.hsv = None
        #self.color = self.change_color("orange")
        self.color_lower_bound = 40
        self.color_upper_bound = 60

        # FPS
        self.last_FPS_time = 0
        self.fps_counter = 0
        self.fps = 0

        # Flags
        self.flag_start_tracking = False
        self.flag_show_thresh = False
        self.flag_start_motors = False

        # initialize spinboxes
        self.init_spinboxes()
        self.init_servo_boxes()
        # set default color
        self.color = self.change_slider_value("orange")
        # start cam and timer
        self.start_cam()


    def start_cam(self):
        self.capture = cv2.VideoCapture(1)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_height)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(0)

    def update_frame(self):
        if self.fps_counter == 0:
           self.last_FPS_time = time.time()  # start fps counter

        self.fps_counter += 1
        if self.fps_counter >= 20:
            self.fps = int(self.fps_counter / (time.time() - self.last_FPS_time))
            self.fps_counter = 0  # reset fps counter
            #print(self.fps)
        ret, self.frame = self.capture.read()   # read image
        self.frame = self.frame[0:480, 0:480]   # crop image
        cv2.putText(self.frame, str(self.fps), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)   # print FPS
        self.process_frame(self.frame)

    def process_frame(self, frame):
        self.hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)             # transform into HSV color space
        blurred = cv2.GaussianBlur(self.hsv, (11,11), 0)              # blur it
        mask = cv2.inRange(blurred, self.color_low, self.color_high)  # threshold it
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find countours in the mask
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)


            # only proceed if the radius meets a minimum size
            if radius > 10:
                # cv2.putText(frame, str(int(x)) + "," + str(int(y)), (int(x) - 50, int(y) - 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
                if self.flag_start_tracking:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (255, 255, 255), 2)        # put a circle around the contour
                    cv2.circle(frame, (int(x), int(y)), 5, (255, 255, 255), -1)                 # put a circle to show center of contour
                if self.flag_start_motors:
                    # draw setpoint on screen
                    cv2.circle(frame, (int(self.setPointX), int(self.setPointY)), 5, (0, 0, 255), -1)
                    self.PID(self.setPointX, self.setPointY, x, y)
        if self.flag_show_thresh == True:
            self.frame = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        self.display_image(self.frame)

    def display_image(self, img):
        rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytesPerLine = ch * w
        img_qt = QImage(rgb_image.data, w, h, bytesPerLine, QImage.Format_RGB888)
        self.cam_label.setPixmap(QPixmap.fromImage(img_qt))

    def PID(self, setPointX, setPointY, inputX, inputY):
        # how long since last calculated
        now = time.time()
        # change in time
        dT = now - self.last_time
        # print(dT)
        # save for next iteration
        self.last_time = now

        # error variables
        errorX = self.setPointX - inputX
        errorY = self.setPointY - inputY


        # integral
        self.errorSumX += errorX * dT
        self.errorSumY += errorY * dT
        # limit x integral
        if self.errorSumX > self.max_error_sum:
            self.errorSumX = self.max_error_sum
        elif self.errorSumX < - self.max_error_sum:
            self.errorSumX = - self.max_error_sum

        # limit y integral
        if self.errorSumY > self.max_error_sum:
            self.errorSumY = self.max_error_sum
        elif self.errorSumY < - self.max_error_sum:
            self.errorSumY = - self.max_error_sum

        # print(self.errorSumX)
        # print(self.max_error_sum)
        dErrorX = (errorX - self.lastErrorX) / dT
        dErrorY = (errorY - self.lastErrorY) / dT

        angleX = self.zero_x + (errorX * self.kP + dErrorX * self.kD + self.kI * self.errorSumX)
        angleY = self.zero_y + (errorY * self.kP + dErrorY * self.kD + self.kI * self.errorSumY)

        # limit output
        if angleX > self.zero_x + 15:
            angleX = self.zero_x + 15
        elif angleX < self.zero_x - 15:
            angleX = self.zero_x - 15
        if angleY > self.zero_y + 15:
            angleY = self.zero_y + 15
        elif angleY < self.zero_y - 15:
            angleY = self.zero_y - 15

        angleX = round(angleX, 1)      
        angleY = round(angleY, 1)

        # remember errors
        self.lastErrorX = errorX
        self.lastErrorY = errorY

        # send to arduino
        arduino.write((str(angleX) + "," + str(angleY) + "\n").encode())
        # print(angleX, angleY)

    def start_tracking(self):
        if self.flag_start_tracking == False:
            self.flag_start_tracking = True
            self.button_start_tracking.setText("Stop Tracking")
        else:
            self.flag_start_tracking = False
            self.button_start_tracking.setText("Start Tracking")


    def start_motors(self):
        if self.flag_start_motors == False:
            self.flag_start_motors = True
            self.button_start_motors.setText("Stop Motors")
        else:
            self.flag_start_motors = False
            self.button_start_motors.setText("Start Motors")
            arduino.write((str(self.zero_x) + "," + str(self.zero_y) + "\n").encode())


    def show_thresh(self):
        if self.flag_show_thresh == False:
            self.flag_show_thresh = True
            self.button_show_thresh.setText("Normal View")
        else:
            self.flag_show_thresh = False
            self.button_show_thresh.setText("Show Thresh")


    def change_slider_value(self, color):
        loc_color = self.my_colors[color]

        self.h_min_slider.setValue(loc_color[0][0])
        self.s_min_slider.setValue(loc_color[0][1])
        self.v_min_slider.setValue(loc_color[0][2])
        self.h_max_slider.setValue(loc_color[1][0])
        self.s_max_slider.setValue(loc_color[1][1])
        self.v_max_slider.setValue(loc_color[1][2])
        self.label_h_min.setText(str(loc_color[0][0]))
        self.label_s_min.setText(str(loc_color[0][1]))
        self.label_v_min.setText(str(loc_color[0][2]))
        self.label_h_max.setText(str(loc_color[1][0]))
        self.label_s_max.setText(str(loc_color[1][1]))
        self.label_v_max.setText(str(loc_color[1][2]))



    def update_color(self):
        self.color_low = (
            self.h_min_slider.value(), self.s_min_slider.value(),
            self.v_min_slider.value())  # get current slider position
        self.color_high = (
            self.h_max_slider.value(), self.s_max_slider.value(),
            self.v_max_slider.value())  # get current slider position

        self.label_h_min.setText(str(self.color_low[0]))
        self.label_s_min.setText(str(self.color_low[1]))
        self.label_v_min.setText(str(self.color_low[2]))
        self.label_h_max.setText(str(self.color_high[0]))
        self.label_s_max.setText(str(self.color_high[1]))
        self.label_v_max.setText(str(self.color_high[2]))


    def mouse_event(self, event):
        x = event.x()
        y = event.y()
        if event.buttons() == Qt.LeftButton:
            self.setPointX = x
            self.setPointY = y
            print(x,y)
        elif event.buttons() == Qt.RightButton:
            hsv = self.hsv
            #print(hsv[x,y])
            self.h_min_slider.setValue(hsv[x, y][0] - 15)
            self.s_min_slider.setValue(hsv[x, y][1] - 80)
            self.v_min_slider.setValue(hsv[x, y][2] - 80)
            self.h_max_slider.setValue(hsv[x, y][0] + 15)
            self.s_max_slider.setValue(hsv[x, y][1] + 80)
            self.v_max_slider.setValue(hsv[x, y][2] + 80)


    def update_spinbox(self):
        self.kP = self.p_box.value()
        self.kI = self.i_box.value()
        self.kD = self.d_box.value()
        self.max_error_sum = self.max_int_box.value()

    def init_spinboxes(self):
        self.p_box.setValue(self.default_P)
        self.i_box.setValue(self.default_I)
        self.d_box.setValue(self.default_D)
    def init_servo_boxes(self):
        self.x_box.setValue(self.zero_y)
        self.y_box.setValue(self.zero_x)

    def reset_spinboxes(self):
        self.init_spinboxes()


    def update_start_angle(self):
        self.zero_x = self.x_box.value()
        self.zero_y = self.y_box.value()
        arduino.write((str(self.zero_x) + "," + str(self.zero_y) + "\n").encode())
        #print(self.zero_x, self.zero_y)

if __name__ == "__main__":
    arduino = serial.Serial('COM3', baudrate=19200, timeout=1)
    app = QApplication(sys.argv)
    window = GUI()
    window.setWindowTitle("Ball Tracking")
    window.show()
    sys.exit(app.exec_())
