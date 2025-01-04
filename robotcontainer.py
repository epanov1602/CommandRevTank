#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations
import typing

from wpilib import XboxController
from wpimath.geometry import Pose2d
import wpilib

from commands2 import InstantCommand, RunCommand, Command
from commands2.button import JoystickButton
import commands2

from subsystems.driveSubsystem import DriveSubsystem
from commands.arcadedrive import ArcadeDrive
from commands.reset_xy import ResetXY

import constants


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, subsystems, and button mappings) should be declared here.
    """

    def __init__(self):
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        # The driver's controller.
        self.driverController = XboxController(constants.kDriverControllerPort)

        # Configure the button bindings
        self.configureButtons()

        # Configure default subsystems
        # Set the default drive command to split-stick arcade drive
        self.robotDrive.setDefaultCommand(ArcadeDrive(
            self.robotDrive,
            lambda: -self.driverController.getLeftY(),
            lambda: -self.driverController.getLeftX(),
        ))

        # Another way to do it would be:
        #self.robotDrive.setDefaultCommand(
        #    # A split-stick arcade command, with forward/backward controlled by the left
        #    # hand, and turning controlled by the right.
        #    RunCommand(
        #        lambda: self.robotDrive.arcadeDrive(
        #            -self.driverController.getLeftY(),
        #            -self.driverController.getLeftX(),
        #        ),
        #        self.robotDrive,
        #    )
        #)

    def configureButtons(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a GenericHID or one of its subclasses (Joystick or XboxController),
        and then calling passing it to a JoystickButton.
        """

        # example 1: reset odometry when the "left bumper" is clicked
        leftBumper = JoystickButton(self.driverController, XboxController.Button.kLeftBumper)
        leftBumper.onTrue(ResetXY(0.0, 0.0, 0.0, drivetrain=self.robotDrive))

        # example 2: drive at half speed when the "right bumper" button is held
        rightBumper = JoystickButton(self.driverController, XboxController.Button.kRightBumper)
        rightBumper.onTrue(InstantCommand(lambda: self.robotDrive.setMaxOutput(0.5)))
        rightBumper.onFalse(InstantCommand(lambda: self.robotDrive.setMaxOutput(1)))


    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureAutos(self):
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        self.chosenAuto.setDefaultOption("trajectory example", self.getAutonomousTrajectoryExample)
        self.chosenAuto.addOption("left blue", self.getAutonomousLeftBlue)
        self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getAutonomousLeftBlue(self):
        setStartPose = ResetXY(x=0.783, y=6.686, headingDegrees=+60, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(1.0)).andThen(stop)
        return command

    def getAutonomousLeftRed(self):
        setStartPose = ResetXY(x=15.777, y=4.431, headingDegrees=-120, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
