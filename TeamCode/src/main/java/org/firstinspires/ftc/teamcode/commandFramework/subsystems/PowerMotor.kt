package org.firstinspires.ftc.teamcode.commandFramework.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.commandFramework.Command

class PowerMotor(
    private val motor: DcMotorSimple,
    private val power: Double,
    private val mode: DcMotor.RunMode? = null
) : Command() {

    override fun start() {
        if (mode != null && motor is DcMotor) {
            motor.mode = mode
        }
        motor.power = power
    }
}