package org.firstinspires.ftc.teamcode.commandFramework.example.opmode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.opModes.TeleOpOpMode
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants
import org.firstinspires.ftc.teamcode.main.subsystems.drive.OdometryConstants

@Disabled
@TeleOp(name = "Example Auto OpMode")
class ExampleTeleOpOpMode : TeleOpOpMode(
    null,
    null,
    MecanumDrive(
        DriveConstants,
        TwoWheelOdometryLocalizer(OdometryConstants),
        Pose2d()
    )
)