package org.firstinspires.ftc.teamcode.main.opmodes.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.main.other.TrajectoryFactory

/**
 * This object performs several tasks that need to be done at the start of every competition OpMode.
 * @param color the color of the alliance
 * @param startPose the starting position of the robot. For information on
 *                  how to use the coordinate system, go to TrajectoryFactory
 */
@Suppress("unused")
abstract class AutonomousOpMode(val color: Constants.Color,
                                val startPose: (() -> Pose2d),
                                private vararg val subsystems: Subsystem,
                                private val commands: List<Command> = listOf()
) : LinearOpMode() {

    override fun runOpMode() {
        Constants.opMode = this
        Constants.color = color
        TrajectoryFactory.initialize()
        Constants.drive.poseEstimate = startPose.invoke()
        CommandScheduler.registerSubsystems(*subsystems)
        commands.forEach { CommandScheduler.scheduleCommand(it) }
        while (!isStarted && !isStopRequested) {
            CommandScheduler.run()
        }
    }
}