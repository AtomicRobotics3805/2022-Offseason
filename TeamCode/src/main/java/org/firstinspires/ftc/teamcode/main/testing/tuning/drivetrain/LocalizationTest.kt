package org.firstinspires.ftc.teamcode.main.testing.tuning.drivetrain

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants.drive
import org.firstinspires.ftc.teamcode.commandFramework.Constants.opMode
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
class LocalizationTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        opMode = this
        CommandScheduler.scheduleCommand(drive.driverControlled(gamepad1))
        waitForStart()
        while (!isStopRequested) {
            CommandScheduler.run()
        }
    }
}