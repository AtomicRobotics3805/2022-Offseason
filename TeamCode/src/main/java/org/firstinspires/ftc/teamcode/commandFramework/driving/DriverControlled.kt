package org.firstinspires.ftc.teamcode.util.commands.driving

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants.drive
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem

/**
 * Controls the robot manually using a gamepad. Left stick up/down moves the robot forwards/backwards, left stick left/
 * right moves the robot left/right, right stick left/right makes the robot turn left/right
 * 
 * @param gamepad the gamepad that controls the driving
 * @param driveSubsystem the subsystem that drives the robot
 * @param reverseStrafe whether to reverse the left/right direction
 * @param reverseStraight whether to reverse the forwards/backwards direction
 * @param reverseTurn whether to reverse the turning left/right direction
 */
class DriverControlled(private val gamepad: Gamepad,
                       driveSubsystem: Subsystem,
                       private val reverseStrafe: Boolean = true,
                       private val reverseStraight: Boolean = true,
                       private val reverseTurn: Boolean = true) : Command() {
    
    override val _isDone = false
    override val requirements: List<Subsystem> = arrayListOf(driveSubsystem)

    /**
     * Calculates and sets the robot's drive power
     */
    override fun execute() {
        val drivePower = Pose2d(
                if (reverseStraight) {-1} else {1} * (gamepad.left_stick_y).toDouble(),
                if (reverseStrafe) {-1} else {1} * (gamepad.left_stick_x).toDouble(),
                if (reverseTurn) {-1} else {1} * (gamepad.right_stick_x).toDouble()
        )

        drive.setWeightedDrivePower(drivePower * drive.driverSpeed)
    }
}