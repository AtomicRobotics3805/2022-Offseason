package org.firstinspires.ftc.teamcode.main.opmodes.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.opModes.AutonomousOpMode
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants
import org.firstinspires.ftc.teamcode.main.subsystems.drive.OdometryConstants

class ExampleAutonomousOpMode : AutonomousOpMode(
    Constants.Color.BLUE,

    { Pose2d() },
    MecanumDrive(
        DriveConstants,
        TwoWheelOdometryLocalizer(OdometryConstants),
        startPose.invoke()
    ),
)