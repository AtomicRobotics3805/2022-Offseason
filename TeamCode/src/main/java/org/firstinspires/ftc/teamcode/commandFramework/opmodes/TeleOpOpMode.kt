package org.firstinspires.ftc.teamcode.commandFramework.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.TelemetryController
import org.firstinspires.ftc.teamcode.commandFramework.controls.Controls
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.TrajectoryFactory

/**
 * This object performs several tasks that need to be done at the start of every competition OpMode.
 * @param color the color of the alliance
 * @param startPose the starting position of the robot. For information on
 *                  how to use the coordinate system, go to TrajectoryFactory
 */
@Suppress("unused")
abstract class TeleOpOpMode(private val controls: Controls,
                            private val color: Constants.Color = Constants.Color.UNKNOWN,
                            private val trajectoryFactory: TrajectoryFactory? = null,
                            private val mainRoutine: (() -> Command)? = null,
                            private val initRoutine: (() -> Command)? = null,
                            private vararg val subsystems: Subsystem
) : LinearOpMode() {

    override fun runOpMode() {
        try {
            // setting constants
            Constants.opMode = this
            if (Constants.color == Constants.Color.UNKNOWN)
                Constants.color = color
            // initializing trajectory factory
            if (trajectoryFactory != null && !trajectoryFactory.initialized)
                trajectoryFactory.initialize()
            // controls stuff
            controls.registerGamepads()
            controls.registerCommands()
            // this both registers & initializes the subsystems
            CommandScheduler.registerSubsystems(TelemetryController, *subsystems)
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