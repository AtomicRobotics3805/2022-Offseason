package org.firstinspires.ftc.teamcode.commandFramework.tuning.drivetrain

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.Constants.drive
import org.firstinspires.ftc.teamcode.commandFramework.TelemetryController
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.MecanumDriveWheelLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.sequential
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectory
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.toRadians
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.Delay
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
class SplineTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        Constants.opMode = this
        drive = MecanumDrive(
            DriveConstants,
            { MecanumDriveWheelLocalizer(drive as MecanumDrive) },
            Pose2d()
        )
        CommandScheduler.registerSubsystems(drive, TelemetryController)
        val forwardTrajectory: ParallelTrajectory = drive.trajectoryBuilder(Pose2d())
            .splineTo(Vector2d(30.0, 30.0), 0.0)
            .build()
        val reverseTrajectory: ParallelTrajectory = drive.trajectoryBuilder(forwardTrajectory.end(), true)
            .splineTo(Vector2d(0.0, 0.0), 180.0.toRadians)
            .build()
        waitForStart()
        CommandScheduler.scheduleCommand(sequential {
            +drive.followTrajectory(forwardTrajectory)
            +Delay(2.0)
            +drive.followTrajectory(reverseTrajectory)
        })
        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}