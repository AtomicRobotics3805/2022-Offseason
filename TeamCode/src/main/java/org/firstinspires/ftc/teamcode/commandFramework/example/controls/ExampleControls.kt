package org.firstinspires.ftc.teamcode.commandFramework.example.controls

import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.controls.Controls
import org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms.Lift

object ExampleControls : Controls() {

    override fun registerCommands() {
        // gamepad 1
        CommandScheduler.scheduleCommand(Constants.drive.driverControlled(Constants.opMode.gamepad1))
        gamepad1.a.startCommand = { Constants.drive.switchSpeed() }
        // gamepad 2
        gamepad2.dpadUp.startCommand = { Lift.toHigh }
        gamepad2.dpadDown.startCommand = { Lift.toLow }
        gamepad2.leftBumper.startCommand = { Lift.start }
        gamepad2.leftBumper.releasedCommand = { Lift.stop }
        gamepad2.rightBumper.startCommand = { Lift.reverse }
        gamepad2.rightBumper.releasedCommand = { Lift.stop }
    }
}