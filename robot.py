import wpilib
from wpilib.drive import MecanumDrive
from wpilib import DigitalInput, DigitalOutput
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import ReplanningConfig, PIDConstants




from rev import CANSparkMax, CANSparkLowLevel, RelativeEncoder


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):

        # Joysticks
        self.joystick = wpilib.Joystick(0)
        self.controller = wpilib.GenericHID(1)  
        self.drive_speed = 0.8

        # Mecanum Drive Motors
        self.top_left = CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless)
        self.bottom_left = CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless)
        self.top_right = CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless)
        self.bottom_right = CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless)
        self.climber_motor = CANSparkMax(8, CANSparkLowLevel.MotorType.kBrushless)

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

        ready_shoot = True 
        at_shooter = False

        # Load pre-generated paths
        self.main_path = pplib.load_path("pickup") 
        self.return_path = pplib.load_path("shooter")

   

    def autonomousInit(self):
        self.auto_timer = wpilib.Timer()
        self.auto_timer.start()
        self.ready_shoot = True
        self.ring_spotted = False

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

    def autonomousPeriodic(self):
        if self.ready_shoot: 
            # Follow the return path
            self.follow_trajectory(self.return_path)

            # Check for 'shoot' event marker
            if self.robot.detect_event_marker("shoot"):
                vision_shoot()  
                self.ready_shoot = False

        else: 
            if not self.ring_spotted:  
                # Follow main path
                self.follow_trajectory(self.main_path)
                self.ring_spotted = see_ring()
            else:  
                vision_pickup()  
                self.ready_shoot = True 

        # ... (Rest of your autonomous logic)





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
