import wpilib
from wpilib.drive import MecanumDrive
from wpilib import DigitalInput, DigitalOutput
import wpilib.cameraserver
import cv2
import numpy as np
import pyAprilTag
from rev import CANSparkMax, CANSparkMaxLowLevel


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):

        # Joysticks
        self.joystick = wpilib.Joystick(0)
        self.controller = wpilib.GenericHID(1)  
        self.drive_speed = 0.8

        # Mecanum Drive Motors
        self.top_left = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_left = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.top_right = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_right = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.climber_motor = CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless)

        self.mec_drive = MecanumDrive(
            self.top_left, self.bottom_left, self.top_right, self.bottom_right
        )

        # Invert motors if needed
        self.top_right.setInverted(True)
        self.bottom_right.setInverted(True)  # Adjust inversions as needed

        # Additional Talon Motors
        self.motor_left = CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless)  # Replace 10 with actual ID
        self.motor_right = CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless)  # Replace 11 with actual ID

        # Ultrasonic Sensors
        self.sensor1_trig = DigitalOutput(0)  # Adjust pin numbers as needed
        self.sensor1_echo = DigitalInput(1)
        self.sensor2_trig = DigitalOutput(2)
        self.sensor2_echo = DigitalInput(3)

        distance1 = self.get_distance(self.sensor1_trig, self.sensor1_echo)
        distance2 = self.get_distance(self.sensor2_trig, self.sensor2_echo)

        # Camera
        self.camera = wpilib.cameraserver.USBCamera("RingDetector", 0)
        self.camera.setResolution(320, 240)  # Adjust resolution if needed
        self.cvSink = wpilib.cameraserver.CvSink("CameraCvSink")
        
        self.cvSink.setSource(self.camera)
        self.tag_detector = pyAprilTag.Detector()


    def autonomousInit(self):
        # Reset any timers or tracking variables if needed
        self.ring_search_timer = wpilib.Timer()
        self.ring_search_timer.start()
        self.ring_found = False  
        self.tag_found = False


    def teleopPeriodic(self):
        """Code for controlling mecanum drive and additional motors"""

        # Mecanum Drive
        x = self.joystick.getX()
        y = self.joystick.getY() 
        rotation = self.joystick.getZ()
        self.mec_drive.driveCartesian(y*self.drive_speed, x*self.drive_speed, rotation*self.drive_speed, 0.0)  # 4th argument is field-oriented angle

        # Additional Talon Motors
        if self.controller.getRawButton(5):  # Left Bumper
            self.motor_left.set(0.5)  # Replace with desired control logic
        else:
            self.motor_left.set(0)

        if self.controller.getRawButton(6):  # Right Bumper
            self.motor_right.set(0.5)  # Replace with desired control logic 
        else:
            self.motor_right.set(0)
        #climber    
        if self.controller.getRawButton(4):  
            self.climber_motor.set(0.8)
        elif self.controller.getRawButton(2): 
            self.climber_motor.set(-0.8)
        else:
            self.climber_motor.set(0)

        # cams
        success, frame = self.cvSink.grabFrame(frame) 
        if success:
            # Replace with your color thresholding or object detection logic
            processed_image = self.process_image(frame)

            # Replace with your distance estimation logic
            estimated_distance = self.estimate_distance(processed_image)

            # Print the distance for now
            print("Estimated Distance to Ring:", estimated_distance)

            # Use the estimated_distance for robot control, e.g.,
            if estimated_distance < 2.0:  # 2 meters example
                # Trigger some action 
                pass
    def autonomousPeriodic(self):
        # Stage 1: Find the orange ring
        if not self.ring_found:
            self.search_and_approach_ring()

        # Stage 2: Turn and find the AprilTag
        elif not self.tag_found:
            self.turn_and_locate_tag()

        # Stage 3: Approach the AprilTag (maintaining distance)
        elif self.tag_found:
            self.approach_tag()

        # Stage 4: Shoot
        else:
            self.shoot()

    # --- Helper Functions for each Stage ---
    def search_and_approach_ring(self):
        # Use camera + OpenCV to detect ring (from previous example)
        # ...

        # If ring detected:
            # Drive towards the ring
            # needs to be modified as drive should be vector
            if estimated_distance > 1.0: 
                self.mec_drive.driveCartesian(0.3, 0, 0)  # Adjust speed
            else:
                self.ring_found = True
                # Activate intake motor briefly
                self.intake_motor.set(0.5)
                wpilib.Timer.delay(1.0)  # Time of intake
                self.intake_motor.set(0.0)

    def turn_and_locate_tag(self):
        # Turn gradually until AprilTag is detected
        self.mec_drive.driveCartesian(0, 0, 0.2)  # Rotation speed

        success, frame = self.cvSink.grabFrame(frame) 
        if success:
            detections = self.tag_detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
            # ... Check if tag is detected ...
            if tag_detected:
                self.tag_found = True

    def approach_tag(self):
        # Use ultrasonic sensors for collision avoidance

        # If no obstacles AND distance to tag > 2 feet:
            self.mec_drive.driveCartesian(0.2, 0, 0)  # Adjust speed as needed
        else:
            # Stop 
            self.mec_drive.driveCartesian(0, 0, 0)

    def shoot(self):
        self.shoot_motor.set(1.0)
        wpilib.TImer.delay(2.0)  # Time needed for shooting
        self.shoot_motor.set(0.0)
    def process_image(self, image):
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define orange color range (adjust these values!)
        lower_orange = np.array([5, 100, 100])  
        upper_orange = np.array([20, 255, 255])

        # Create a mask for the orange color
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Find contours of the orange objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours  # Return contours for further analysis

    def estimate_distance(self, contours):
        KNOWN_RING_DIAMETER = 0.254
        KNOWN_FOCAL_LENGTH = 320 * KNOWN_RING_DIAMETER / ring_width  # Assume width and focal length related

        # Find a suitable ring contour (you might add more criteria)
        for contour in contours:
            # Approximate contour with a circle 
            (x, y), radius = cv2.minEnclosingCircle(contour)  

            # Estimate distance only if the radius is within a reasonable range 
            if radius > 20 and radius < 100: 
                ring_width = 2 * radius  # Approximate width in pixels
                distance = KNOWN_FOCAL_LENGTH * KNOWN_RING_DIAMETER / ring_width
                return distance

        return None  # No suitable ring found        

    def get_distance(self, trig_pin, echo_pin):
        """Calculates distance using an HC-SR04 sensor"""
        trig_pin.value = True    # Send a 10us pulse 
        wpilib.Timer.delay(0.00001)
        trig_pin.value = False

        while echo_pin.value == 0:  # Wait for the Echo pin to go high
            pulse_start = wpilib.Timer.getFPGATimestamp()

        while echo_pin.value == 1:  # Wait for Echo pin to go low
            pulse_end = wpilib.Timer.getFPGATimestamp()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150.0  # Convert duration to cm
        return distance
