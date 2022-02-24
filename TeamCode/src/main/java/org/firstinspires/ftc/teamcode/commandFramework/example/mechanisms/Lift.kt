package org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.commandFramework.Constants.opMode
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.CustomCommand
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import kotlin.math.abs

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object Arm : Subsystem {
    @JvmField
    var ARM_NAME = "lift"
    @JvmField
    var ARM_SPEED = 1.0
    @JvmField
    var ARM_POSITION_HIGH = 0.0 // in
    @JvmField
    var ARM_POSITION_LOW = 0.0 // in

    private const val PULLEY_WIDTH = 1.0 // in
    private const val COUNTS_PER_REV = 537.6
    // higher value makes driven gear slower
    private const val DRIVE_GEAR_REDUCTION = 1.0
    private const val COUNTS_PER_INCH = COUNTS_PER_REV * DRIVE_GEAR_REDUCTION / (PULLEY_WIDTH * Math.PI)

    val toLow: Command
        get() = moveArmToPosition((ARM_POSITION_LOW * COUNTS_PER_INCH).toInt())
    val toHigh: Command
        get() = moveArmToPosition((ARM_POSITION_HIGH * COUNTS_PER_INCH).toInt())
    val toStart: Command
        get() = moveArmToPosition((0.1 * COUNTS_PER_INCH).toInt())
    val start: Command
        get() = powerArm(ARM_SPEED)
    val reverse: Command
        get() = powerArm(-ARM_SPEED)
    val stop: Command
        get() = halt()

    lateinit var armMotor: DcMotorEx

    fun initialize() {
        armMotor = opMode.hardwareMap.get(DcMotorEx::class.java, ARM_NAME)
        armMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

    }

    fun halt() = CustomCommand(_start = {
        armMotor.power = 0.0
    })

    fun powerArm(speed: Double) = CustomCommand(_start = {
        armMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        armMotor.power = speed
    })

    fun moveArmToPosition(position: Int) = CustomCommand(
            getDone = { abs(armMotor.targetPosition - armMotor.currentPosition) <= armMotor.targetPositionTolerance },
            _start = {
                armMotor.targetPosition = position
                armMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                armMotor.power = ARM_SPEED
            },
            _done = {
                if (position == 0)
                    armMotor.power = 0.0
            }
    )
}