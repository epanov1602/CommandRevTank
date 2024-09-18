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

class DriveSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        # The motors on the left side of the drive.
        motorL1 = rev.CANSparkMax(constants.kLeftMotor1CAN, rev.CANSparkMax.MotorType.kBrushless)
        motorL2 = rev.CANSparkMax(constants.kLeftMotor2CAN, rev.CANSparkMax.MotorType.kBrushless)
        motorL1.setInverted(False)  # TODO: this may or may not need to be inverted in your case -- check!
        motorL2.setInverted(False)  # TODO: this may or may not need to be inverted in your case -- check!
        self.leftMotors = MotorControllerGroup(motorL1, motorL2)

        # The motors on the right side of the drive.
        motorR1 = rev.CANSparkMax(constants.kRightMotor1CAN, rev.CANSparkMax.MotorType.kBrushless)
        motorR2 = rev.CANSparkMax(constants.kRightMotor2CAN, rev.CANSparkMax.MotorType.kBrushless)
        motorR1.setInverted(False)  # TODO: this may or may not need to be inverted in your case -- check!
        motorR2.setInverted(True)  # TODO: this may or may not need to be inverted in your case -- check!
        self.rightMotors = MotorControllerGroup(motorR1, motorR2)

        # The robot's drive
        self.drive = DifferentialDrive(self.leftMotors, self.rightMotors)

        # The left-side drive encoder
        self.leftEncoder = motorL1.getEncoder()

        # The right-side drive encoder
        self.rightEncoder = motorR1.getEncoder()

        # The gyro sensor
        self.gyro = navx.AHRS.create_spi()
        sleep(1.0)  # wait until gyro recalibrates, this takes 1s

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightMotors.setInverted(True)

        # Sets the distance per pulse for the encoders
        self.leftEncoder.setPositionConversionFactor(constants.kEncoderPositionConversionFactor)
        self.rightEncoder.setPositionConversionFactor(constants.kEncoderPositionConversionFactor)

        self.odometry = DifferentialDriveOdometry(
            self.gyro.getRotation2d(),
            self.leftEncoder.getPosition() * constants.kLeftEncoderSign,
            self.rightEncoder.getPosition() * constants.kRightEncoderSign,
        )

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

    def arcadeDrive(self, fwd, rot):
        """Drives the robot using arcade controls."""
        self.drive.arcadeDrive(fwd, rot)

    def tankDriveVolts(self, leftVolts, rightVolts):
        """Controls the left and right sides of the drive directly with voltages."""
        self.leftMotors.setVoltage(leftVolts)
        self.rightMotors.setVoltage(rightVolts)
        self.drive.feed()

    def getAverageEncoderDistance(self):
        """Gets the average distance of the two encoders."""
        return (self.leftEncoder.getPosition() * constants.kLeftEncoderSign +
                self.rightEncoder.getPosition() * constants.kRightEncoderSign) / 2

    def setMaxOutput(self, maxOutput):
        """Sets the max output of the drive. Useful for scaling the drive to drive more slowly."""
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
