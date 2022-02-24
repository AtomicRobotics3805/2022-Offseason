package org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.commandFramework.Constants.opMode
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.CustomCommand
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Intake : Subsystem {
    @JvmField
    var INTAKE_NAME = "intake"
    @JvmField
    var INTAKE_SPEED = 1.0

    private const val COUNTS_PER_MOTOR_REV = 537.6
    // higher value makes driven gear slower
    private const val DRIVE_GEAR_REDUCTION = 1.0
    private const val ROTATION_DEGREES = 500
    private const val ROTATION_COUNTS = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION *
            (ROTATION_DEGREES / 360.0)).toInt()

    val start: Command
        get() = powerIntake(INTAKE_SPEED)
    val stop: Command
        get() = powerIntake(0.0)
    val switch: Command
        get() = if (on) stop else start

    var on = false
    private lateinit var intakeMotor: DcMotorEx

    fun initialize() {
        intakeMotor = opMode.hardwareMap.get(DcMotorEx::class.java, INTAKE_NAME)
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor.direction = DcMotorSimple.Direction.REVERSE
    }

    fun powerIntake(power: Double) = CustomCommand(_start = {
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor.power = power
        on = power != 0.0
    })
}