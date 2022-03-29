package org.firstinspires.ftc.teamcode.commandFramework.driving

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants.drive
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.Driver
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem

/**
 * Controls the robot manually using a gamepad. Left stick up/down moves the robot forwards/backwards, left stick left/
 * right moves the robot left/right, right stick left/right makes the robot turn left/right
 *
 * @param gamepad the gamepad that controls the driving
 * @param requirements any Subsystems used by this command
 * @param interruptible whether this command can be interrupted or not
 * @param reverseStrafe whether to reverse the left/right direction
 * @param reverseStraight whether to reverse the forwards/backwards direction
 * @param reverseTurn whether to reverse the turning left/right direction
 */
class DriverControlled(
    private val gamepad: Gamepad,
    override val requirements: List<Subsystem> = arrayListOf(),
    override val interruptible: Boolean = true,
    private val reverseStrafe: Boolean = false,
    private val reverseStraight: Boolean = true,
    private val reverseTurn: Boolean = false
) : Command() {

    override val _isDone = false

    /**
     * Calculates and sets the robot's drive power
     */
    override fun execute() {
        val drivePower = Pose2d(
                if (reverseStraight) -1 else {1} * (gamepad.left_stick_y).toDouble(),
                if (reverseStrafe) -1 else {1} * (gamepad.left_stick_x).toDouble(),
                if (reverseTurn) -1 else {1} * (gamepad.right_stick_x).toDouble()
        )

        drive.setDrivePower(drivePower * drive.driverSpeed)
    }
}