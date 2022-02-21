package org.firstinspires.ftc.teamcode.commandFramework

import com.qualcomm.robotcore.hardware.Gamepad

/**
    CustomGamepad makes it much easier to detect when a button has just been pressed or released
    To use the program, call update(gamepad1) or update(gamepad2) every frame
    Check whether a button is being pressed down using button.down
    Check whether it's just been pressed this frame using button.pressed
    Check whether it's just been released this frame using button.released
 */


@Suppress("MemberVisibilityCanBePrivate")
class CustomGamepad(private val gamepad: Gamepad) {
    val a = Button("A")
    val b = Button("B")
    val x = Button("X")
    val y = Button("Y")

    val dpadUp = Button("D-Pad Up")
    val dpadDown = Button("D-Pad Down")
    val dpadLeft = Button("D-Pad Left")
    val dpadRight = Button("D-Pad Right")

    val leftBumper = Button("Left Bumper")
    val rightBumper = Button("Right Bumper")

    val leftTrigger = Trigger("Left Trigger")
    val rightTrigger = Trigger("Right Trigger")

    val leftStick = JoyStick("Left Stick")
    val rightStick = JoyStick("Right Stick")

    val controls = listOf(a, b, x, y, dpadUp, dpadDown, dpadLeft, dpadRight,
        leftBumper, rightBumper, leftTrigger, rightTrigger, leftStick, rightStick)

    fun update(gamepad: Gamepad = this.gamepad) {
        a.update(gamepad.a)
        b.update(gamepad.b)
        x.update(gamepad.x)
        y.update(gamepad.y)

        dpadUp.update(gamepad.dpad_up)
        dpadDown.update(gamepad.dpad_down)
        dpadLeft.update(gamepad.dpad_left)
        dpadRight.update(gamepad.dpad_right)

        leftBumper.update(gamepad.left_bumper)
        rightBumper.update(gamepad.right_bumper)

        leftTrigger.update(gamepad.left_trigger)
        rightTrigger.update(gamepad.right_trigger)

        leftStick.update(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.left_stick_button)
        rightStick.update(gamepad.right_stick_x, gamepad.right_stick_y, gamepad.right_stick_button)
    }

    override fun toString(): String {
        val set = mutableSetOf<String>()

        controls.forEach {
            if(it.toString() != "")
                set.add(it.toString())
        }

        return set.joinToString("\n")
    }

    class Button(private val name: String = "Unknown Button") {

        var down = false
        var pressed = false
        var released = false
        var startCommand: (() -> Command)? = null
        var releasedCommand: (() -> Command)? = null

        fun update(value: Boolean) {
            // Update whether or not the button is pressed, released, or down
            pressed = value && !down
            released = !value && down
            down = value

            // Run the associated commands
            pressedCommand.update(value)
            releasedCommand.update(value)
            if (pressed && startCommand != null) {
                CommandScheduler.scheduleCommand(startCommand)
            }
            if (released && releasedCommand != null) {
                CommandScheduler.scheduleCommand(releaseCommand)
            }
        }

        override fun toString(): String {
            val set = mutableSetOf<String>()
            if (down) set.add("Down")
            if (pressed) set.add("Just Pressed")
            if (released) set.add("Just Released")
            return if (set.isNotEmpty()) "$name: ${set.joinToString(", ")}" else ""
        }
    }

    class Trigger(private val name: String = "Unknown Trigger") {
        val down: Boolean
            get() = amount != 0.0f
        var pressed = false
        var released = false
        var amount = 0.0f
        var startCommand: (() -> Command)? = null
        var releasedCommand: (() -> Command)? = null

        fun update(value: Float) {
            pressed = value != 0.0f && !down
            released = value == 0.0f && down
            amount = value
            if (pressed && startCommand != null) {
                CommandScheduler.scheduleCommand(startCommand)
            }
            if (released && releasedCommand != null) {
                CommandScheduler.scheduleCommand(releaseCommand)
            }
        }

        override fun toString(): String {
            val set = mutableSetOf<String>()
            if (pressed) set.add("Just Pressed")
            if (released) set.add("Just Released")
            if (amount != 0.0f) set.add("Amount Pressed: $amount")
            return if (set.isNotEmpty()) "$name: ${set.joinToString(", ")}" else ""
        }
    }

    class JoyStick(private val name: String = "Unknown Joystick") {
        val moved: Boolean
            get() = x != 0.0f || y != 0.0f
        var justMoved = false
        var justStopped = false
        var x = 0.0f
        var y = 0.0f
        val button = Button()

        fun update(x: Float, y: Float, buttonValue: Boolean) {
            justMoved = x != 0.0f || y != 0.0f && !moved
            justStopped = x == 0.0f && y == 0.0f && moved
            this.x = x
            this.y = y
            button.update(buttonValue)
        }

        override fun toString(): String {
            val set = mutableSetOf<String>()
            if (button.down) set.add("Button Down")
            if (button.pressed) set.add("Button Just Pressed")
            if (button.released) set.add("Button Just Released")
            if (justMoved) set.add("Just Started Moving")
            if (justStopped) set.add("Just Stopped Moving")
            if (set.isNotEmpty() || x != 0.0f || y != 0.0f) {
                set.add("X: $x")
                set.add("Y: $y")
            }
            return if (set.isNotEmpty()) "$name: ${set.joinToString(", ")}" else ""
        }
    }
}