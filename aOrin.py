import cv2
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, cudaToNumpy
from motor_movement_ctrl import Roboclaw
import tkinter as tk
import sys
import jetson.utils
import threading

class ObjectTracker:
    def __init__(self):
        # Initialize object detection model
        self.modelName = "tongoRealD&PRangeTest"
        self.modelName = "natFactory"
        self.orinName = "orin2"
        
        self.net = detectNet(argv=["--model=/home/"+ self.orinName +"/git/Weplatt/Rowp/models/" + self.modelName + ".onnx", "--labels=/home/"+ self.orinName +"/git/Weplatt/Rowp/models/labelsDronePerson.txt", "--input-blob=input_0", "--output-cvg=scores", "--output-bbox=boxes"], threshold=0.35)
        
        self.fractionForYaxis = 1/2  # Add this line to initialize fractionForYaxis
        
        # Initialize video source and output
        self.camera = videoSource("/dev/video0")  # '/dev/video0' for V4L2
        self.display = videoOutput()  # 'my_video.mp4' for file
        
        # Initialize Roboclaw motor controllers
        self.roboclawLR = Roboclaw("/dev/ttyACM0", 38400)  # Change this to match your Roboclaw port
        self.roboclawLR.Open()
        
        self.roboclawUD = Roboclaw("/dev/ttyACM1", 38400)
        self.roboclawUD.Open()
        
        # Define parameters and variables
        self.detectionsByClass = {}
        self.personClassID = 3
        
        self.screen_width = self.camera.GetWidth()
        self.screen_height = self.camera.GetHeight()
        self.screen_center_x = self.screen_width / 2
        self.screen_center_y = self.screen_height / 2
        
        self.maxSpeedx = 50
        self.minSpeedx = 7
        
        self.maxSpeedy = 50
        self.minSpeedy = 6
        
        self.tolerance = 15  # 25 person    15 drone
        self.tolerancey = 20  # 15 person  10 drone
        
        self.pwm_flag_LR = False
        self.pwm_flag_UD = False
        self.address = 0x80

        # Initialize Tkinter
        self.root = tk.Tk()
        self.root.title("Class Tracking Selection")
        self.root.wm_attributes("-topmost", True)
        self.create_buttons()
        self.root.after(100, self.update_gui)

    def switch_tracking(self, class_id):
        self.personClassID = class_id
        print(class_id)
        if class_id == 1:  # DRONE CLASS
            self.fractionForYaxis = 1 / 2
            self.tolerance = 15
            self.tolerancey = 10
        elif class_id == 2:  # HUMAN CLASS
            self.fractionForYaxis = 1 / 3
            self.tolerance = 20
            self.tolerancey = 20
            
    def create_buttons(self):
        for class_id in range(1, 4):  # Assuming classes range from 1 to 3
            class_label = f"Track Class {class_id}"
            button = tk.Button(self.root, text=class_label, command=lambda id=class_id: self.switch_tracking(id), width=30, height=6)
            button.pack()

            # Bind keyboard events
            self.root.bind(str(class_id), lambda event, id=class_id: self.switch_tracking(id))

    def update_gui(self):
        self.root.update_idletasks()
        self.root.update()
        
    def object_detection_motor_control(self):
        while self.display.IsStreaming():
            img = self.camera.Capture()

            if img is None:  # capture timeout
                continue

            numpy_img = cudaToNumpy(img)
            detections = self.net.Detect(img)
            self.detectionsByClass.clear()

            for detection in detections:
                class_id = detection.ClassID
                confidence = detection.Confidence
                left = detection.Left
                top = detection.Top
                right = detection.Right
                bottom = detection.Bottom
                center_x = (left + right) / 2
                center_y = top + self.fractionForYaxis * (bottom - top)
                area = (right - left) * (bottom - top)

                if class_id not in self.detectionsByClass:
                    self.detectionsByClass[class_id] = []

                self.detectionsByClass[class_id].append((confidence, left, top, right, bottom, center_x, center_y, area))

            if self.personClassID in self.detectionsByClass:
                detectionsList = self.detectionsByClass[self.personClassID]
                closest_detection = max(detectionsList, key=lambda x: x[7])

                confidence, left, top, right, bottom, center_x, center_y, area = closest_detection

                cv2.rectangle(numpy_img, (int(left), int(top)), (int(right), int(bottom)), (255, 0, 0), 3)

                x_direction = "LEFT" if center_x > (self.screen_center_x + self.tolerance) else "RIGHT"
                y_direction = "UP" if center_y > (self.screen_center_y + self.tolerance) else "DOWN"

                if abs(center_x - self.screen_center_x) > self.tolerance:
                    scaledSpeedx = int((abs(center_x - self.screen_center_x) / self.screen_center_x) * self.maxSpeedx)                    
                    scaledSpeedx = min(max(scaledSpeedx, 0), 63)  # Clamp within [min, 63]
                    print(scaledSpeedx)
                    self.roboclawLR.ForwardBackwardM2(self.address, 63 + scaledSpeedx if x_direction == "LEFT" else 63 - scaledSpeedx)
                    self.pwm_flag_LR = True
                    
                elif self.pwm_flag_LR:
                    self.roboclawLR.BackwardM2(self.address, 0)
                    self.pwm_flag_LR = False

                if abs(center_y - self.screen_center_y) > self.tolerancey:
                    scaledSpeedy = int((abs(center_y - self.screen_center_y) / self.screen_center_y) * self.maxSpeedy)
                    scaledSpeedy = min(max(scaledSpeedy, 0), 63)  # Clamp within [min, 63]
                    self.roboclawUD.ForwardBackwardM1(self.address, 63 + scaledSpeedy if y_direction == "UP" else 63 - scaledSpeedy)
                    self.pwm_flag_UD = True
                elif self.pwm_flag_UD:
                    self.roboclawUD.BackwardM1(self.address, 0)
                    self.pwm_flag_UD = False
            else:
                if self.pwm_flag_LR:
                    self.roboclawLR.BackwardM2(self.address, 0)
                    self.pwm_flag_LR = False
                if self.pwm_flag_UD:
                    self.roboclawUD.BackwardM1(self.address, 0)
                    self.pwm_flag_UD = False

            self.display.Render(img)
            self.display.SetStatus("Object Detection | Network {:.0f} FPS".format(self.net.GetNetworkFPS()))

    def start(self):
        # Start the asynchronous object detection and motor control thread
        object_detection_thread = threading.Thread(target=self.object_detection_motor_control)
        object_detection_thread.daemon = True
        object_detection_thread.start()

        # Main loop
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            self.root.destroy()
            sys.exit(0)

if __name__ == "__main__":
    tracker = ObjectTracker()
    tracker.start()

