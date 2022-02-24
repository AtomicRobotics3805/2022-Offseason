package org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.MoveServo
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem

/**
 * This class is an example of a claw controlled by a single servo. Its first two commands, open and close, which each
 * move the claw to the corresponding position. The third command, switch, opens it if it's closed and closes it if it's
 * open. The switch command is particularly useful during TeleOp.
 * To use this class, copy it into the proper package and change the four constants.
 */
@Suppress("PropertyName", "MemberVisibilityCanBePrivate", "unused")
class Claw : Subsystem {

    // constants
    @JvmField
    var CAP_CLAW_NAME = "claw"
    @JvmField
    var OPEN_POSITION = 0.0
    @JvmField
    var CLOSE_POSITION = 1.0
    // the number of seconds required to move the servo from 0.0 to 1.0
    @JvmField
    var SPEED = 1.0

    // commands
    val open: Command
        get() = MoveServo(clawServo, OPEN_POSITION, SPEED, listOf(this), true)
    val close: Command
        get() = MoveServo(clawServo, CLOSE_POSITION, SPEED, listOf(this), true)
    val switch: Command
        get() = if (clawServo.position == OPEN_POSITION) close else open

    // servo
    private lateinit var clawServo: Servo

    /**
     * Initializes the clawServo.
     */
    fun initialize() {
        clawServo = Constants.opMode.hardwareMap.get(Servo::class.java, CAP_CLAW_NAME)
    }
}