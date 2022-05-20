package org.firstinspires.ftc.teamcode.commandFramework.tuning.drivetrain

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.Constants.drive
import org.firstinspires.ftc.teamcode.commandFramework.TelemetryController
import org.firstinspires.ftc.teamcode.commandFramework.driving.Turn
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.MecanumDriveWheelLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.toRadians
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
class TurnTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        Constants.opMode = this
        Constants.drive = MecanumDrive(
            DriveConstants,
            { MecanumDriveWheelLocalizer(drive as MecanumDrive) },
            Pose2d()
        )
        CommandScheduler.registerSubsystems(Constants.drive, TelemetryController)
        waitForStart()
        CommandScheduler.scheduleCommand(Constants.drive.turn(ANGLE.toRadians, Turn.TurnType.RELATIVE))
        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }

    companion object {
        var ANGLE = 90.0 // deg
    }
}