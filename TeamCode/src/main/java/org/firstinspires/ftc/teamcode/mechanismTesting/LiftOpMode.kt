package org.firstinspires.ftc.teamcode.mechanismTesting

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.main.other.Controls

@TeleOp(name = "LiftOpMode Thingy", group = "TestingOpModes")
class LiftOpMode: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this
        LiftMechanism.initialize()
        CommandScheduler.registerSubsystems()
        Controls.registerGamepads()
        Controls.registerCommands()
        waitForStart()
        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}