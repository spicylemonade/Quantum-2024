from wpilib import DriverStation, MotorControllerGroup, SPI
from pathplannerlib.auto import AutoBuilder
from commands2 import Subsystem
from pathplannerlib.config import ReplanningConfig, PIDConstants
import navx as Gyro
from wpilib.drive import DifferentialDrive
from rev import CANSparkMax, CANSparkLowLevel, RelativeEncoder
from robotConstants import Constants
from wpimath.kinematics import DifferentialDriveOdometry,ChassisSpeeds

from wpimath.geometry import Pose2d, Pose3d
class DriveSubsystem(Subsystem):
    navx = Gyro.AHRS(SPI.Port.kMXP)
    def __init__(self):
        # Do all subsystem initialization here
        self.top_left = CANSparkMax(1, CANSparkLowLevel.MotorType.kBrushless)
        self.bottom_left = CANSparkMax(2, CANSparkLowLevel.MotorType.kBrushless)
        self.top_right = CANSparkMax(4, CANSparkLowLevel.MotorType.kBrushless)
        self.bottom_right = CANSparkMax(3, CANSparkLowLevel.MotorType.kBrushless)
        self.climber_motor = CANSparkMax(8, CANSparkLowLevel.MotorType.kBrushless)

        self.leftEncoder = self.top_left.getEncoder()
        self.rightEncoder = self.top_right.getEncoder()

        self.leftGroup = MotorControllerGroup(self.top_left, self.bottom_left)
        self.rightGroup = MotorControllerGroup(self.top_right, self.bottom_right)

        self.difDrive = DifferentialDrive(self.leftGroup, self.rightGroup)

        self.top_left.restoreFactoryDefaults()
        self.bottom_left.restoreFactoryDefaults()
        self.top_right.restoreFactoryDefaults()
        self.bottom_right.restoreFactoryDefaults()


        self.leftEncoder.setPosition(0)
        self.rightEncoder.setPosition(0)

        self.rightEncoder.setPositionConversionFactor(Constants.DriveTrainConstants.mechLinearDistanceConversionFactor)
        self.leftEncoder.setPositionConversionFactor(Constants.DriveTrainConstants.mechLinearDistanceConversionFactor)
        self.rightEncoder.setVelocityConversionFactor(Constants.DriveTrainConstants.mechLinearDistanceConversionFactor / 60)
        self.leftEncoder.setVelocityConversionFactor(Constants.DriveTrainConstants.mechLinearDistanceConversionFactor / 60)
        self.bottom_left.follow(self.top_left)
        self.bottom_right.follow(self.top_right)

        self.rightGroup.setInverted(True)
        self.rightGroup.setInverted(False)



        self.m_odometry = DifferentialDriveOdometry(self.navx.getRotation2d,self.leftEncoder.getPosition(), self.rightEncoder.getPosition())
        self.m_odometry.resetPosition(Pose2d(),self.navx.getRotation2d())

        # ...
    

        # Configure the AutoBuilder last
        AutoBuilder.configureRamsete(
            self.getPose, # Robot pose supplier
            self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getCurrentSpeeds, # Current ChassisSpeeds supplier
            self.drive, # Method that will drive the robot given ChassisSpeeds
            ReplanningConfig(), # Default path replanning config. See the API for the options here
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )
    def getPose(self)-> Pose2d:
        return self.m_odometry.getPose()
    def resetPose(self, pose:Pose2d)->None:
        self.leftEncoder.setPosition(0)
        self.rightEncoder.setPosition(0)

        self.m_odometry.resetPosition(pose, self.navx.getRotation2d())
    def getCurrentSpeeds(self):
        return ChassisSpeeds(self.navx.getVelocityX,self.navx.getVelocityY,-self.navx.getRate())
    def periodic(self):
        self.m_odometry.update(self.navx.getRotation2d(),self.leftEncoder.getPosition(),self.rightEncoder.getPosition())

    def shouldFlipPath():
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed