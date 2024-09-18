#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from wpilib import XboxController
from wpimath.geometry import Pose2d

from commands2 import InstantCommand, RunCommand
from commands2.button import JoystickButton

from subsystems.driveSubsystem import DriveSubsystem
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
        self.robotDrive.setDefaultCommand(
            # A split-stick arcade command, with forward/backward controlled by the left
            # hand, and turning controlled by the right.
            RunCommand(
                lambda: self.robotDrive.arcadeDrive(
                    -self.driverController.getLeftY(),
                    -self.driverController.getLeftX(),
                ),
                self.robotDrive,
            )
        )

    def configureButtons(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a GenericHID or one of its subclasses (Joystick or XboxController),
        and then calling passing it to a JoystickButton.
        """

        # example 1: reset odometry when the "left bumper" is clicked
        leftBumper = JoystickButton(self.driverController, XboxController.Button.kLeftBumper)
        leftBumper.onTrue(InstantCommand(lambda: self.robotDrive.resetOdometry(Pose2d(0.0, 0.0, 0.0))))

        # example 2: drive at half speed when the "right bumper" button is held
        rightBumper = JoystickButton(self.driverController, XboxController.Button.kRightBumper)
        rightBumper.onTrue(InstantCommand(lambda: self.robotDrive.setMaxOutput(0.5)))
        rightBumper.onFalse(InstantCommand(lambda: self.robotDrive.setMaxOutput(1)))

    def getAutonomousCommand(self):
        """Use this to pass the autonomous command to the main {@link Robot} class."""
