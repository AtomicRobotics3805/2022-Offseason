package org.firstinspires.ftc.teamcode.commandFramework.example.opmode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.opModes.AutonomousOpMode
import org.firstinspires.ftc.teamcode.commandFramework.sequential
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants
import org.firstinspires.ftc.teamcode.main.subsystems.drive.OdometryConstants

/**
 * This is an example OpMode for the Autonomous (not driver-controlled) period. In a real project,
 * you would have to remove the @Disabled annotation, change the name, and change most of the
 * constructor parameters.
 */
@Disabled
@Autonomous(name = "Example Auto OpMode")
class ExampleAutonomousOpMode : AutonomousOpMode(
    Constants.Color.BLUE,
    { sequential { } },
    null,
    MecanumDrive(
        DriveConstants,
        TwoWheelOdometryLocalizer(OdometryConstants),
        Pose2d()
    )
)