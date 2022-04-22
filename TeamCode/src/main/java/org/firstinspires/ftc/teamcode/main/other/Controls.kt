package org.firstinspires.ftc.teamcode.main.other

import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.controls.Controls
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.sequential
import org.firstinspires.ftc.teamcode.main.mechanisms.*

/**
 * This class manages the controls for TeleOp OpModes. It's currently just a skeleton of what it
 * would be during a competition, since there are no real commands to register. If you want to
 * register a command, type a line into registerCommands with the following format:
 * gamepad.button.command = Subsystem.command
 * For example, that could look like this:
 * gamepad1.a.startCommand = Lift.toHigh
 * If you used the line above, then whenever you pressed the a button on gamepad1 it would move the
 * lift to the high position.
 */
@Suppress("unused")
object Controls : Controls() {

    /**
     * Registers commands on the gamepads. Currently empty since this class is a skeleton.
     * Directions for registering commands are in the class docs.
     */
    override fun registerCommands() {
        CommandScheduler.scheduleCommand(Constants.drive.driverControlled(Constants.opMode.gamepad1))

        // Toggle between driving speeds
        gamepad1.a.startCommand = { Constants.drive.switchSpeed() }

        // Deposit freight routine
        gamepad1.leftBumper.startCommand = { sequential {
            +Lift.toHigh
            +Bucket.drop
            +Lift.toStart
            +Bucket.collect
        }}

        // Intake controls
        gamepad1.b.startCommand = { Intake.start }
        gamepad1.b.releasedCommand = { Intake.stop }
        gamepad2.rightTrigger.startCommand = { Intake.start }
        gamepad2.rightTrigger.releasedCommand = { Intake.stop }
        gamepad2.leftTrigger.startCommand = { Intake.reverse }
        gamepad2.leftTrigger.releasedCommand = { Intake.stop }

        // Carousel controls
        gamepad2.leftBumper.startCommand = { Carousel.start }
        gamepad2.leftBumper.releasedCommand = { Carousel.stop }
        gamepad2.rightBumper.startCommand = { Carousel.reverse }
        gamepad2.rightBumper.releasedCommand = { Carousel.stop }

        // Bucket controls
        gamepad2.x.startCommand = { Bucket.collect }
        gamepad2.y.startCommand = { Bucket.drop }
        gamepad2.a.startCommand = { BucketLock.close }
        gamepad2.b.startCommand = { BucketLock.open }

        // Lift controls
        gamepad2.dpadDown.startCommand = { Lift.toStart }
        gamepad2.dpadLeft.startCommand = { Lift.toLow }
        gamepad2.dpadUp.startCommand = { Lift.toMiddle }
        gamepad2.dpadRight.startCommand = { Lift.toHigh }
        gamepad1.dpadUp.startCommand = { Lift.raise }
        gamepad1.dpadUp.releasedCommand = { Lift.stop }
        gamepad1.dpadRight.startCommand = { Lift.toHigh }
        gamepad1.dpadLeft.startCommand = { Lift.toLow }
    }
}