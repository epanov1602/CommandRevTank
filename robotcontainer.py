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
from commands2.button import CommandGenericHID
import commands2

from commands.trajectory import JerkyTrajectory
from subsystems.drivesubsystem import DriveSubsystem, BadSimPhysics
from commands.arcadedrive import ArcadeDrive
from commands.reset_xy import ResetXY

import constants
from subsystems.limelight_camera import LimelightCamera
from subsystems.limelight_localizer import LimelightLocalizer


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, subsystems, and button mappings) should be declared here.
    """

    def __init__(self, robot):
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()
        self.limelightLocalizer = LimelightLocalizer(self.robotDrive)

        #self.frontCamera = LimelightCamera("limelight-front")

        #self.limelightLocalizer.addCamera(
        #    self.frontCamera,
        #    cameraPoseOnRobot=Translation3d(x=0.40, y=-0.15, z=0.5),
        #    cameraHeadingOnRobot=Rotation2d.fromDegrees(0.0))

        # The driver's controller.
        self.driverController = CommandGenericHID(constants.kDriverControllerPort)

        # Configure the button bindings
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default subsystems
        # Set the default drive command to split-stick arcade drive
        self.robotDrive.setDefaultCommand(ArcadeDrive(
            lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftY),
            lambda: -self.driverController.getRawAxis(XboxController.Axis.kLeftX),
            self.robotDrive,
            assumeManualInput=True,
        ))

        if commands2.TimedCommandRobot.isSimulation():
            self.robotDrive.simPhysics = BadSimPhysics(self.robotDrive, robot)

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a GenericHID or one of its subclasses (Joystick or XboxController),
        and then calling passing it to a JoystickButton.
        """

        # example 2: when "POV-up" button pressed, reset robot field position to "facing North"
        resetFacingNorthCommand = ResetXY(x=1.0, y=4.0, headingDegrees=0, drivetrain=self.robotDrive)
        povUpButton = self.driverController.povUp()
        povUpButton.whileTrue(resetFacingNorthCommand)

        # example 3: when "POV-down" is pressed, reset robot field position to "facing South"
        resetFacingSouthCommand = ResetXY(x=7.0, y=4.0, headingDegrees=180, drivetrain=self.robotDrive)
        povDownButton = self.driverController.povDown()
        povDownButton.whileTrue(resetFacingSouthCommand)

        # example 4: robot drives this trajectory command when "A" button is pressed
        trajectoryCommand1 = JerkyTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                # format: (x, y, heading)
                (1.0, 7.0, -54),  # start at left feeding station: x=1.0, y=7.0, heading=-54 degrees
                (1.25, 6.75, -54),  # next waypoint
                (1.5, 6.50, -54),  # next waypoint
                (1.9, 6.0, -54),  # next waypoint
                (1.9, 4.0, 0),  # next waypoint
                (2.2, 4.0, 0),  # next waypoint
                (2.7, 4.0, 0),  # next waypoint
            ],
            endpoint=(3.2, 4.0, 0),  # end point at the reef facing North
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )
        aButton = self.driverController.button(XboxController.Button.kA)
        aButton.whileTrue(trajectoryCommand1)  # while "A" button is pressed, keep running trajectoryCommand1

        # example 5: and when "B" button is pressed, drive the reversed trajectory
        reversedTrajectoryCommand1 = trajectoryCommand1.reversed()
        bButton = self.driverController.button(XboxController.Button.kB)
        bButton.whileTrue(reversedTrajectoryCommand1)  # while "B" button is pressed, keep running this command


    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureAutos(self):
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        self.chosenAuto.setDefaultOption("left blue", self.getAutonomousLeftBlue)
        self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getAutonomousLeftBlue(self):
        setStartPose = ResetXY(x=0.783, y=6.686, headingDegrees=+60, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(1.0, 0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(1.0)).andThen(stop)
        return command

    def getAutonomousLeftRed(self):
        setStartPose = ResetXY(x=15.777, y=4.431, headingDegrees=-120, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(1.0, 0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
