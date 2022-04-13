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
 * All Competition TeleOp OpModes should inherit from this class. It performs several tasks that all
 * TeleOp programs need to do. Each competition OpMode only needs to pass parameters to the
 * constructor - it doesn't need to have a body.
 * @param mainRoutine a lambda returning the main routine that needs to be performed after the play
 *                    button is pressed. This routine should be declared in a separate Routines
 *                    class. It's not necessary if there aren't any automatic tasks that you want to
 *                    perform at the start of TeleOp.
 * @param initRoutine a lambda returning the routine that needs to be performed during
 *                    initialization. This routine should also be declared in a separate Routines
 *                    class. It should not move the robot.
 * @param subsystems each of the subsystems used by the robot. One of these should be a drivetrain
 *                   and the others should be mechanisms.
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
            // this both registers & initializes the subsystems
            CommandScheduler.registerSubsystems(TelemetryController, *subsystems)
            // if there is a routine that's supposed to be performed on init, then do it
            if (initRoutine != null) CommandScheduler.scheduleCommand(initRoutine.invoke())
            // wait for start
            while (!isStarted && !isStopRequested) {
                CommandScheduler.run()
            }
            // commands have to be registered after the subsystems are registered
            controls.registerCommands()
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