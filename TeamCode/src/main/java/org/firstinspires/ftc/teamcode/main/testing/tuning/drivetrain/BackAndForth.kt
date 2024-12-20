package org.firstinspires.ftc.teamcode.main.testing.tuning.drivetrain

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commandFramework.*
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectory
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants
import org.firstinspires.ftc.teamcode.main.subsystems.drive.OdometryConstants

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
class BackAndForth : LinearOpMode() {

    private lateinit var trajectoryForward: ParallelTrajectory
    private lateinit var trajectoryBackward: ParallelTrajectory
    private val followTrajectories: Command
        get() = sequential {
            +Constants.drive.followTrajectory(trajectoryForward)
            +Constants.drive.followTrajectory(trajectoryBackward)
        }

    override fun runOpMode() {
        Constants.opMode = this
        Constants.drive = MecanumDrive(
            DriveConstants,
            TwoWheelOdometryLocalizer(OdometryConstants),
        ) { Pose2d() }
        CommandScheduler.registerSubsystems(Constants.drive, TelemetryController)
        trajectoryForward = Constants.drive.trajectoryBuilder(Pose2d())
            .forward(DISTANCE)
            .build()
        trajectoryBackward = Constants.drive.trajectoryBuilder(trajectoryForward.end())
            .back(DISTANCE)
            .build()
        waitForStart()
        CommandScheduler.scheduleCommand(followTrajectories)
        while (opModeIsActive()) {
            if (!CommandScheduler.hasCommands()) {
                CommandScheduler.scheduleCommand(followTrajectories)
            }
            CommandScheduler.run()
        }
    }

    companion object {
        @JvmField
        var DISTANCE = 50.0
    }
}