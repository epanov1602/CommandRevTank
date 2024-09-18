#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

"""
A place for the constant values in the code that may be used in more than one place.
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""

from wpimath.kinematics import DifferentialDriveKinematics

import math

# ID for the driver's joystick.
kDriverControllerPort = 0

# The CAN IDs for the drivetrain motor controllers.
kLeftMotor1CAN = 1
kLeftMotor2CAN = 2
kRightMotor1CAN = 3
kRightMotor2CAN = 4

# Encoders and their respective motor controllers.
kLeftEncoderSign = +1
kRightEncoderSign = -1  # reversed

# In meters, distance between wheels on each side of robot.
kTrackWidthMeters = 0.69
kDriveKinematics = DifferentialDriveKinematics(kTrackWidthMeters)

# Encoder counts per revolution/rotation.
kEncoderCPR = 1024
kWheelDiameterMeters = 0.15

# Please calibrate to your robot
kEncoderPositionConversionFactor = 0.7
