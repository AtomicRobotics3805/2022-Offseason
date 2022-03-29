package org.firstinspires.ftc.teamcode.main.opmodes.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.TelemetryController
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.main.other.Controls
import org.firstinspires.ftc.teamcode.main.other.TrajectoryFactory
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants
import org.firstinspires.ftc.teamcode.main.subsystems.drive.OdometryConstants

/**
 * This object performs several tasks that need to be done at the start of every competition OpMode.
 * @param color the color of the alliance
 * @param startPose the starting position of the robot. For information on
 *                  how to use the coordinate system, go to TrajectoryFactory
 */
@Suppress("unused")
abstract class TeleOpOpMode(private val color: Constants.Color,
                            private vararg val subsystems: Subsystem,
                            private val startPose: (() -> Pose2d)? = null,
                            private val mainRoutine: (() -> Command)? = null,
                            private val initRoutine: (() -> Command)? = null
) : LinearOpMode() {

    override fun runOpMode() {
        try {
            // setting constants
            Constants.opMode = this
            CommandScheduler.registerSubsystems(
                MecanumDrive(
                    DriveConstants,
                    TwoWheelOdometryLocalizer(OdometryConstants),
                    startPose?.invoke() ?: Pose2d()
                ), *subsystems)
            // controls stuff
            Controls.registerGamepads()
            Controls.registerCommands()
            // this both registers & initializes the subsystems
            CommandScheduler.registerSubsystems(*subsystems)
            // if there is a routine that's supposed to be performed on init, then do it
            if (initRoutine != null) CommandScheduler.scheduleCommand(initRoutine.invoke())
            // wait for start
            while (!isStarted && !isStopRequested) {
                CommandScheduler.run()
            }
            // if there's a routine that's supposed to be performed on play, do it
            if (mainRoutine != null) CommandScheduler.scheduleCommand(mainRoutine.invoke())
            // wait for stop
            while (opModeIsActive()) {
                CommandScheduler.run()
            }
        } catch (e: Exception) {
            // we have to catch the exception so that we can still cancel and unregister
            TelemetryController.telemetry.addLine("Error Occurred!")
            TelemetryController.telemetry.addLine(e.message)
            // have to update telemetry since CommandScheduler won't call it anymore
            TelemetryController.periodic()
        } finally {
            // cancels all commands and unregisters all gamepads & subsystems
            CommandScheduler.cancelAll()
            CommandScheduler.unregisterAll()
        }
    }
}