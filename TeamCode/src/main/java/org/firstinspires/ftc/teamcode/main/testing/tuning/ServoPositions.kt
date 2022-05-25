package org.firstinspires.ftc.teamcode.main.testing.tuning

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants.opMode
import org.firstinspires.ftc.teamcode.commandFramework.GamepadEx
import org.firstinspires.ftc.teamcode.commandFramework.TelemetryController

/**
 * This class tunes the position of a servo. Before you use it, make sure to change SERVO_NAME.
 * Move the left stick on gamepad1 to the left or right to spin the servo, then note the position
 * on the telemetry. Once you've gotten it to the correct position, put that into your Subsystem
 * object. Repeat this process for however many different positions/servos you need to test.
 */
@Suppress("unused", "PropertyName")
@TeleOp(name = "Servo Position Tuning", group = "Test")
class ServoPositions : LinearOpMode() {

    @JvmField
    var SERVO_NAME = "servo"

    /**
     * Main function, uses other classes like TuneServoCommand to run the OpMode
     */
    override fun runOpMode() {
        opMode = this
        val servo = hardwareMap.get(Servo::class.java, SERVO_NAME)
        val gamepad = GamepadEx(gamepad1)
        waitForStart()
        CommandScheduler.scheduleCommand(TuneServoCommand(servo, gamepad.leftStick))
        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }

    /**
     * As the name implies, this command tunes the positions of a servo. It's designed for use in
     * the ServoPositions OpMode, although it can also be used elsewhere. It works by measuring the
     * amount of time since the last loop and moving the servo an amount based on that time. That
     * way, the servo speed isn't affected by the loop speed.
     *
     * @param servo the servo to move
     * @param joyStick the joy stick that controls the servo
     */
    class TuneServoCommand(private val servo: Servo, private val joyStick: GamepadEx.JoyStick) :
        Command() {

        override val _isDone = false
        private var position = 0.5
        private var oldTime = 0.0
        private val timer = ElapsedTime()

        /**
         * Resets the timer
         */
        override fun start() {
            timer.reset()
        }

        /**
         * Calculates the change in time since the last loop and increases/decreases the servo
         * position based on that. Clips the servo position between 0.0 and 1.0, and puts the
         * position on telemetry.
         */
        override fun execute() {
            val deltaTime = timer.seconds() - oldTime
            position = Range.clip(position + joyStick.x * deltaTime, 0.0, 1.0)
            servo.position = position
            TelemetryController.telemetry.addData("Servo Position", position)
        }
    }
}

