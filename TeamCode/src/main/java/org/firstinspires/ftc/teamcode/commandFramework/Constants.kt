package org.firstinspires.ftc.teamcode.commandFramework

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode

object Constants {

    enum class Color {
        BLUE,
        RED
    }
    lateinit var color: Color
    lateinit var opMode: LinearOpMode
    lateinit var drive: Driver
}