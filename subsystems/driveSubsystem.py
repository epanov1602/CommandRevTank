#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import Subsystem

from wpilib import MotorControllerGroup, ADXRS450_Gyro
from wpilib.drive import DifferentialDrive
from wpilib import SmartDashboard

from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveWheelSpeeds

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
    def __init__(self, usePIDController=True):
        super().__init__()

        # The motors on the left side of the drive.
        self.motorL1 = rev.CANSparkMax(constants.kLeftMotor1CAN, rev.CANSparkMax.MotorType.kBrushless)
        self.motorL2 = rev.CANSparkMax(constants.kLeftMotor2CAN, rev.CANSparkMax.MotorType.kBrushless)
        self.motorL1.setInverted(False)  # TODO: this may or may not need to be inverted in your case -- test!
        self.motorL2.setInverted(False)  # TODO: this may or may not need to be inverted in your case -- test!

        # The motors on the right side of the drive.
        self.motorR1 = rev.CANSparkMax(constants.kRightMotor1CAN, rev.CANSparkMax.MotorType.kBrushless)
        self.motorR2 = rev.CANSparkMax(constants.kRightMotor2CAN, rev.CANSparkMax.MotorType.kBrushless)
        self.motorR1.setInverted(False)  # TODO: this may or may not need to be inverted in your case -- test!
        self.motorR2.setInverted(False)  # TODO: this may or may not need to be inverted in your case -- test!

        if usePIDController:
            # do not use basic differential drive, take advantage of low-level PID control from Rev
            self.drive = None
            self.leftMotors = None
            self.rightMotors = None
            self.leftPIDController = self.motorL1.getPIDController()
            self.rightPIDController = self.motorR1.getPIDController()
            self.motorL2.follow(self.motorL1, invert=False)  # TODO: this may or may not need to be inverted in your case -- test!
            self.motorR2.follow(self.motorR1, invert=False)  # TODO: this may or may not need to be inverted in your case -- test!
        else:
            # use the basic differential drive (robot will be less responsive and slower)
            # We need to invert one side of the drivetrain so that positive voltages
            # result in both sides moving forward. Depending on how your robot's
            # gearbox is constructed, you might have to invert the left side instead.
            self.leftMotors = MotorControllerGroup(self.motorL1, self.motorL2)
            self.rightMotors = MotorControllerGroup(self.motorR1, self.motorR2)
            self.rightMotors.setInverted(True)
            self.drive = DifferentialDrive(self.leftMotors, self.rightMotors)

        # The left-side drive encoder
        self.leftEncoder = self.motorL1.getEncoder()

        # The right-side drive encoder
        self.rightEncoder = self.motorR1.getEncoder()

        # The gyro sensor
        self.gyro = navx.AHRS.create_spi()
        sleep(1.0)  # wait until gyro recalibrates, this takes 1s

        # Sets the distance per pulse for the encoders
        self.leftEncoder.setPositionConversionFactor(constants.kEncoderPositionConversionFactor)
        self.rightEncoder.setPositionConversionFactor(constants.kEncoderPositionConversionFactor)

        self.odometry = DifferentialDriveOdometry(
            self.gyro.getRotation2d(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
        )

        SmartDashboard.setDefaultNumber("driveKPMult", 0.5)
        SmartDashboard.setDefaultNumber("driveKDMult", 0.5)
        SmartDashboard.setDefaultNumber("driveKFFMult", 1.0)
        SmartDashboard.setDefaultNumber("driveMaxSpeedMult", 1.0)
        SmartDashboard.setDefaultNumber("driveMaxAccMult", 1.0)
        self._setupMotorConfigs()

    def stop(self):
        if self.drive:
            self.drive.stopMotor()
        else:
            self.leftPIDController.setReference(0, rev.CANSparkBase.ControlType.kVelocity)
            self.rightPIDController.setReference(0, rev.CANSparkBase.ControlType.kVelocity)
            self._setupMotorConfigs()


    def periodic(self):
        # Update the odometry in the periodic block
        self.odometry.update(
            self.gyro.getRotation2d(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
        )
        # Update the pose of the robot (x, y, heading) on the SmartDashboard
        pose = self.getPose()
        SmartDashboard.putNumber("x", pose.x)
        SmartDashboard.putNumber("y", pose.y)
        SmartDashboard.putNumber("heading", pose.rotation().degrees())

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
        self.odometry.resetPosition(
            self.gyro.getRotation2d(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
            pose,
        )

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
            self.leftPIDController.setReference(left, rev.CANSparkBase.ControlType.kVelocity)
            self.rightPIDController.setReference(right, rev.CANSparkBase.ControlType.kVelocity)

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
        return self.gyro.getRotation2d().degrees()

    def getTurnRate(self):
        """Returns the turn rate of the robot."""
        return -self.gyro.getRate()

    def _setupMotorConfigs(self):
        if self.drive is not None:
            return  # self.drive is responsible for this
        # otherwise, we are controlling individual motors using PID
        self.motorL1.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        self.motorL2.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        self.motorR1.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        self.motorR2.setIdleMode(rev.CANSparkBase.IdleMode.kBrake)
        self._setupMotorConfig(self.leftPIDController)
        self._setupMotorConfig(self.rightPIDController)

    def _setupMotorConfig(self, pidController):
        # set PID coefficients
        kFFMult = SmartDashboard.getNumber("driveKFFMult", 1.0)
        kDMult = SmartDashboard.getNumber("driveKDMult", 0.5)
        kPMult = SmartDashboard.getNumber("driveKPMult", 0.5)
        if kPMult < 0:
            kPMult = 0  # safety
        if kPMult > 4:
            kPMult = 4  # safety

        pidController.setP(kPMult * DrivetrainConstants.initialP)
        pidController.setD(kDMult * DrivetrainConstants.initialD)
        pidController.setFF(kFFMult * DrivetrainConstants.initialFF)
        pidController.setIZone(0)
        pidController.setI(0)
        pidController.setOutputRange(-1, +1)
