#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from __future__ import annotations

import typing
import commands2
from subsystems.drivesubsystem import DriveSubsystem


class ArcadeDrive(commands2.Command):
    def __init__(
        self,
        driveSpeed: typing.Callable[[], float] | float,
        rotationSpeed: typing.Callable[[], float] | float,
        drivetrain: DriveSubsystem,
        assumeManualInput=False,
    ) -> None:
        """Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
        lambdas. This command does not terminate.

        :param drivetrain:  The drivetrain subsystem on which this command will run
        :param driveSpeed:     Callable supplier of forward/backward speed
        :param rotationSpeed:    Callable supplier of rotational speed
        """
        super().__init__()

        self.driveSpeed = driveSpeed
        if not callable(driveSpeed):
            self.driveSpeed = lambda: driveSpeed

        self.rotationSpeed = rotationSpeed
        if not callable(rotationSpeed):
            self.rotationSpeed = lambda: rotationSpeed

        self.assumeManualInput = assumeManualInput
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def initialize(self):
        pass

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        driveSpeed = self.driveSpeed()  # get the drive speed from the joystick or wherever it comes from
        rotationSpeed = self.rotationSpeed()  # get the turn speed from the joystick or wherever it comes from
        self.drivetrain.arcadeDrive(driveSpeed, rotationSpeed, assumeManualInput=self.assumeManualInput)

    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)  # stop at the end
