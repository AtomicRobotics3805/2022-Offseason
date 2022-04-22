package org.firstinspires.ftc.teamcode.main.testing.tuning.drivetrain

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commandFramework.*
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectory
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.TelemetryCommand
import org.firstinspires.ftc.teamcode.main.subsystems.drive.TrioDriveConstants
import org.firstinspires.ftc.teamcode.main.subsystems.drive.OdometryConstants

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
class StrafeTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        Constants.opMode = this
        Constants.drive = MecanumDrive(
            TrioDriveConstants,
            TwoWheelOdometryLocalizer(OdometryConstants),
            Pose2d()
        )
        CommandScheduler.registerSubsystems(Constants.drive, TelemetryController)
        val trajectory: ParallelTrajectory = Constants.drive.trajectoryBuilder(Pose2d())
            .strafeRight(DISTANCE)
            .build()
        waitForStart()
        CommandScheduler.scheduleCommand(sequential {
            +Constants.drive.followTrajectory(trajectory)
            +parallel {
                TelemetryCommand(1000.0, "finalX") { Constants.drive.poseEstimate.x.toString() }
                TelemetryCommand(1000.0, "finalY") { Constants.drive.poseEstimate.y.toString() }
                TelemetryCommand(1000.0, "finalHeading") { Constants.drive.poseEstimate.heading.toString() }
            }
        })
        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }

    companion object {
        var DISTANCE = 60.0 // in
    }
}