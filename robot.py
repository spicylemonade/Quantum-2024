import wpilib
from wpilib.drive import MecanumDrive
from rev import CANSparkMax, CANSparkMaxLowLevel


class MyRobot(wpilib.TimedRobot):
    def __init__(self):
        super().__init__()

        # Joysticks
        self.joystick = wpilib.Joystick(0)
        self.controller = wpilib.GenericHID(1)  

        # Mecanum Drive Motors
        self.top_left = CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_left = CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.top_right = CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless)
        self.bottom_right = CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless)

        self.mec_drive = MecanumDrive(
            self.top_left, self.bottom_left, self.top_right, self.bottom_right
        )

        # Invert motors if needed
        self.top_right.setInverted(True)
        self.bottom_right.setInverted(True)  # Adjust inversions as needed

        # Additional Talon Motors
        self.motor_left = CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless)  # Replace 10 with actual ID
        self.motor_right = CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless)  # Replace 11 with actual ID

    def teleopPeriodic(self):
        """Code for controlling mecanum drive and additional motors"""

        # Mecanum Drive
        x = self.joystick.getX()
        y = self.joystick.getY() 
        rotation = self.joystick.getZ()
        self.mec_drive.driveCartesian(y, x, rotation, 0.0)  # 4th argument is field-oriented angle

        # Additional Talon Motors
        if self.controller.getRawButton(5):  # Left Bumper
            self.motor_left.set(0.5)  # Replace with desired control logic
        else:
            self.motor_left.set(0)

        if self.controller.getRawButton(6):  # Right Bumper
            self.motor_right.set(0.5)  # Replace with desired control logic 
        else:
            self.motor_right.set(0)