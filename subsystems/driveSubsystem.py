#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import Subsystem

from wpilib import MotorControllerGroup, ADXRS450_Gyro
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard, Field2d

from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveWheelSpeeds
from wpimath.geometry import Rotation2d, Pose2d, Translation2d

import constants
import navx
import rev

from time import sleep


class DrivetrainConstants:
    initialP = 2.5 / 10000.0
    initialD = 5.0 / 10000.0  # coincidentally same as initialP, but really does not need to be
    initialFF = 1.4 / 10000.0  # if setting it to nonzero, be careful and start small
    maxRPM = 3000


class DriveSubsystem(Subsystem):
    # noinspection PyInterpreter
    def __init__(self,
                 usePIDController=True,
                 l1MotorInverted=False,
                 l2MotorInverted=False,
                 r1MotorInverted=False,
                 r2MotorInverted=False
    ):
        super().__init__()

        # The motors on the left side of the drive.
        self.motorL1 = rev.SparkMax(constants.kLeftMotor1CAN, rev.SparkMax.MotorType.kBrushless)
        self.motorL1.configure(
            _getLeadMotorConfig(l1MotorInverted, constants.kEncoderPositionConversionFactor),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters)

        self.motorL2 = rev.SparkMax(constants.kLeftMotor2CAN, rev.SparkMax.MotorType.kBrushless)
        self.motorL2.configure(
            _getFollowMotorConfig(constants.kLeftMotor1CAN, l2MotorInverted != l1MotorInverted),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters)

        # The motors on the right side of the drive.
        self.motorR1 = rev.SparkMax(constants.kRightMotor1CAN, rev.SparkMax.MotorType.kBrushless)
        self.motorR1.configure(
            _getLeadMotorConfig(r1MotorInverted, constants.kEncoderPositionConversionFactor),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters)

        self.motorR2 = rev.SparkMax(constants.kRightMotor2CAN, rev.SparkMax.MotorType.kBrushless)
        self.motorR2.configure(
            _getFollowMotorConfig(constants.kRightMotor1CAN, r2MotorInverted != r1MotorInverted),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters)

        if usePIDController:
            # do not use basic differential drive, take advantage of low-level PID control from Rev
            self.drive = None
            self.leftMotors = None
            self.rightMotors = None
            self.leftPIDController = self.motorL1.getClosedLoopController()
            self.rightPIDController = self.motorR1.getClosedLoopController()
        else:
            # use the basic differential drive (robot will be less responsive and slower)
            # We need to invert one side of the drivetrain so that positive voltages
            # result in both sides moving forward. Depending on how your robot's
            # gearbox is constructed, you might have to invert the left side instead.
            self.leftMotors = MotorControllerGroup(self.motorL1, self.motorL2)
            self.rightMotors = MotorControllerGroup(self.motorR1, self.motorR2)
            self.rightMotors.setInverted(True)
            self.drive = DifferentialDrive(self.motorL1, self.motorR1)

        # The left-side drive encoder
        self.leftEncoder = self.motorL1.getEncoder()

        # The right-side drive encoder
        self.rightEncoder = self.motorR1.getEncoder()

        # The gyro sensor
        self.gyro = navx.AHRS.create_spi()
        sleep(1.0)  # wait until gyro recalibrates, this takes 1s

        self.odometry = DifferentialDriveOdometry(
            self.gyro.getRotation2d(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
        )
        self.odometryHeadingOffset = Rotation2d(0)
        self.resetOdometry(Pose2d(0, 0, 0))

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        SmartDashboard.setDefaultNumber("driveKPMult", 0.5)
        SmartDashboard.setDefaultNumber("driveKDMult", 0.5)
        SmartDashboard.setDefaultNumber("driveKFFMult", 1.0)
        SmartDashboard.setDefaultNumber("driveMaxSpeedMult", 1.0)
        SmartDashboard.setDefaultNumber("driveMaxAccMult", 1.0)

    def stop(self):
        if self.drive:
            self.drive.stopMotor()
        else:
            self.leftPIDController.setReference(0, rev.SparkBase.ControlType.kVelocity)
            self.rightPIDController.setReference(0, rev.SparkBase.ControlType.kVelocity)

    def periodic(self):
        # Update the odometry in the periodic block
        pose = self.odometry.update(
            self.gyro.getRotation2d(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
        )
        # Update the pose of the robot (x, y, heading) on the SmartDashboard
        SmartDashboard.putNumber("x", pose.x)
        SmartDashboard.putNumber("y", pose.y)
        SmartDashboard.putNumber("heading", pose.rotation().degrees())
        self.field.setRobotPose(pose)

    def getPose(self):
        """Returns the currently-estimated pose of the robot."""
        return self.odometry.getPose()

    def getWheelSpeeds(self):
        """Returns the current wheel speeds of the robot."""
        return DifferentialDriveWheelSpeeds(
            self.leftEncoder.getVelocity(), self.rightEncoder.getVelocity() * constants.kRightEncoderSign
        )

    def resetOdometry(self, pose):
        """Resets the odometry to the specified pose."""
        self.gyro.reset()
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
            pose,
        )
        self.odometryHeadingOffset = self.odometry.getPose().rotation() - self.getGyroHeading()

    def adjustOdometry(self, dTrans: Translation2d, dRot: Rotation2d):
        pose = self.getPose()
        newPose = Pose2d(pose.translation() + dTrans, pose.rotation() + dRot)
        self.odometry.resetPosition(
            pose.rotation() - self.odometryHeadingOffset,
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
            newPose,
        )

    def drive(self, xSpeed, ySpeed, rot, fieldRelative, rateLimit) -> None:
        assert False, "ERROR: swerve drive not available on this drivetrain"

    def arcadeDrive(self, fwd, rot, assumeManualInput=False):
        """Drives the robot using arcade controls."""

        # use curves to take smoother input from human
        if assumeManualInput:
            fwd = fwd * fwd * fwd
            rot = rot * abs(rot)

        if rot > 1:
            rot = 1
        if rot < -1:
            rot = -1

        if self.drive:
            # use basic DifferentialDrive and don't take advantage of low-level Rev PID controller
            self.drive.arcadeDrive(fwd, rot)
        else:
            # use Rev PID control for better speed and acceleration
            # (but when asked to rotate at speed 0.6, we can only drive forward at speedLimit=1-0.6=0.4)
            speedLimit = max((0, 1 - abs(rot)))
            if fwd > speedLimit:
                fwd = speedLimit
            if fwd < -speedLimit:
                fwd = -speedLimit
            right = (fwd + rot) * DrivetrainConstants.maxRPM
            left = (fwd - rot) * DrivetrainConstants.maxRPM
            self.leftPIDController.setReference(left, rev.SparkBase.ControlType.kVelocity)
            self.rightPIDController.setReference(right, rev.SparkBase.ControlType.kVelocity)

    def getAverageEncoderDistance(self):
        """Gets the average distance of the two encoders."""
        return (self.leftEncoder.getPosition() * constants.kLeftEncoderSign +
                self.rightEncoder.getPosition() * constants.kRightEncoderSign) / 2

    def setMaxOutput(self, maxOutput):
        """Sets the max output of the drive. Useful for scaling the drive to drive more slowly."""
        if self.drive:
            self.drive.setMaxOutput(maxOutput)

    def zeroHeading(self):
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def getHeading(self):
        """Returns the heading of the robot."""
        return self.getPose().rotation()

    def getGyroHeading(self):
        """Returns the heading of the robot."""
        return self.gyro.getRotation2d()


    def getTurnRate(self):
        """Returns the turn rate of the robot."""
        return -self.gyro.getRate()


def _getFollowMotorConfig(leadCanID, inverted):
    config = rev.SparkBaseConfig()
    config.follow(leadCanID, inverted)
    return config


def _getLeadMotorConfig(
    inverted: bool,
    positionFactor: float,
) -> rev.SparkBaseConfig:
    config = rev.SparkBaseConfig()
    config.inverted(inverted)
    config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    config.limitSwitch.forwardLimitSwitchEnabled(False)
    config.limitSwitch.reverseLimitSwitchEnabled(False)
    config.encoder.positionConversionFactor(positionFactor)
    #config.encoder.velocityConversionFactor(positionFactor / 60)  # 60 seconds per minute
    config.closedLoop.pid(DrivetrainConstants.initialP, 0.0, DrivetrainConstants.initialD)
    config.closedLoop.velocityFF(DrivetrainConstants.initialFF)
    config.closedLoop.outputRange(-1, +1)
    return config