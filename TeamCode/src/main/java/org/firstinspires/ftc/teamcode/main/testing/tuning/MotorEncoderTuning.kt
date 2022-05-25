package org.firstinspires.ftc.teamcode.main.testing.tuning

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.commandFramework.*
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.MotorToPosition
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.TelemetryCommand

/**
 * @TODO Write Comments
 */
@Suppress("unused", "PropertyName")
@TeleOp(name = "Motor Encoder Tuning", group = "Test")
class MotorEncoderTuning : LinearOpMode() {

    @JvmField
    var MOTOR_NAME = "motor"
    @JvmField
    var SPEED = 1.0
    @JvmField
    var TARGET_POSITION = 0
    @JvmField
    var KP_INCREMENT = 0.001

    private var kP = 0.005

    /**
     * Main function, scans gamepad to alter kP and run MotorToPosition
     */
    override fun runOpMode() {
        Constants.opMode = this
        val motor = hardwareMap.get(DcMotor::class.java, MOTOR_NAME)
        val gamepad = GamepadEx(gamepad1)
        waitForStart()
        while (opModeIsActive()) {
            if (gamepad.dpadUp.pressed) {
                kP += when {
                    gamepad.x.down -> KP_INCREMENT * 10
                    gamepad.y.down -> KP_INCREMENT / 10
                    else -> KP_INCREMENT
                }
            }
            if (gamepad.dpadUp.pressed) {
                kP -= when {
                    gamepad.x.down -> KP_INCREMENT * 10
                    gamepad.y.down -> KP_INCREMENT / 10
                    else -> KP_INCREMENT
                }
            }
            if (gamepad.a.pressed) {
                CommandScheduler.scheduleCommand(
                    sequential {
                        +MotorToPosition(motor, TARGET_POSITION, SPEED, kP = kP)
                    }
                )
            }
            if (gamepad.b.pressed) {
                CommandScheduler.cancelAll()
            }
            telemetry.addData("Current kP", kP)
            CommandScheduler.run()
        }
    }

    /**
     * Just a simple MotorToPosition command, but also sends out a telemetry message when the
     * command ends containing the time required to get to the correct position & the kP value of
     * this command.
     */
    class TuneMotorKP(motor: DcMotor, targetPosition: Int, speed: Double, kP: Double) :
        MotorToPosition(motor, targetPosition, speed, kP = kP) {

        /**
         * Runs the super end function and schedules a telemetry command with the kP and time.
         */
        override fun end(interrupted: Boolean) {
            super.end(interrupted)
            CommandScheduler.scheduleCommand(
                TelemetryCommand(10000.0, "kP: " + kP + ", time: " + timer.seconds())
            )
        }
    }
}