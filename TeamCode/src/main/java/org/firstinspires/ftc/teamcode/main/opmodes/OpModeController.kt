package org.firstinspires.ftc.teamcode.main.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.main.other.TrajectoryFactory

/**
 * This object performs several tasks that need to be done at the start of every competition OpMode.
 */
@Suppress("unused")
object OpModeController {

    /**
     * This is the main function that performs the necessary tasks prior to the beginning of the
     * main program. This includes setting Constants.opMode and Constants.color, initializing start
     * positions and trajectories, setting the start pose, initializing each subsystem, and
     * registering them with the CommandScheduler. Finally, it runs the CommandScheduler until the
     * OpMode starts.
     * @param opMode the OpMode that's currently active. In most cases, you can write "this"
     * @param color the color of the alliance. For TeleOp, set this to null
     * @param startPose the starting position of the robot. For information on how to use the
     *                  coordinate system, go to TrajectoryFactory. For TeleOp, set this to null
     */
    fun initialize(opMode: LinearOpMode, color: Constants.Color?, startPose: (() -> Pose2d)?) {
        Constants.opMode = opModegit
        if (color != null) Constants.color = color
        initializeSubsystems()
        TrajectoryFactory.initialize()
        if (startPose != null) Constants.drive.poseEstimate = startPose.invoke()
        CommandScheduler.registerSubsystems()
        while (!opMode.isStarted && !opMode.isStopRequested) {
            CommandScheduler.run()
        }
    }

    /**
     * Initializes the robot's Subsystems (like their drivetrain & mechanisms). This is currently
     * empty since a template project has no Subsystems yet.
     */
    private fun initializeSubsystems() {

    }
}