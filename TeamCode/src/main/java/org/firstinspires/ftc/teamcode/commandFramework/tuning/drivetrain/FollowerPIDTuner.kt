package org.firstinspires.ftc.teamcode.commandFramework.tuning.drivetrain

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commandFramework.*
import org.firstinspires.ftc.teamcode.commandFramework.Constants.drive
import org.firstinspires.ftc.teamcode.commandFramework.driving.Turn
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.MecanumDriveWheelLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectory
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.toRadians
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(group = "drive")
class FollowerPIDTuner : LinearOpMode() {

    private var startPose = Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0.0)
    private val followTrajectories: Command
        get() {
            val trajectory: ParallelTrajectory = drive.trajectoryBuilder(startPose)
                .forward(DISTANCE)
                .build()
            startPose = trajectory.end().plus(Pose2d(0.0, 0.0, 90.0.toRadians))
            return sequential {
                +drive.followTrajectory(trajectory)
                +drive.turn(90.0.toRadians, Turn.TurnType.RELATIVE)
            }
        }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        Constants.opMode = this
        drive = MecanumDrive(
            DriveConstants,
            { MecanumDriveWheelLocalizer(drive as MecanumDrive) },
            startPose
        )
        CommandScheduler.registerSubsystems(drive, TelemetryController)
        waitForStart()
        while (opModeIsActive()) {
            if (!CommandScheduler.hasCommands()) {
                CommandScheduler.scheduleCommand(followTrajectories)
            }
            CommandScheduler.run()
        }
    }

    companion object {
        var DISTANCE = 48.0 // in
    }
}