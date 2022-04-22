package org.firstinspires.ftc.teamcode.main.mechanisms

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.commandFramework.Constants.opMode
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.PowerMotor
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.CustomCommand

@Config
@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Intake : Subsystem {

    // configurable constants
    @JvmField
    var NAME = "intake"
    @JvmField
    var SPEED = 1.0
    @JvmField
    var DIRECTION = DcMotorSimple.Direction.FORWARD

    // commands
    val start: Command
        get() = PowerMotor(intakeMotor, SPEED, requirements = listOf(this))
    val stop: Command
        get() = PowerMotor(intakeMotor, 0.0, requirements = listOf(this))
    val reverse: Command
        get() = PowerMotor(intakeMotor, -SPEED, requirements = listOf(this))

    // motor
    private lateinit var intakeMotor: DcMotorEx

    /**
     * Initializes the intakeMotor, sets its mode to RUN_WITHOUT_ENCODER, and sets its direction to the DIRECTION
     * variable.
     */
    override fun initialize() {
        intakeMotor = opMode.hardwareMap.get(DcMotorEx::class.java, NAME)
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor.direction = DIRECTION
    }
}