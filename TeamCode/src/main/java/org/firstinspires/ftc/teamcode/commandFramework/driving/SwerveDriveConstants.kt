package org.firstinspires.ftc.teamcode.commandFramework.driving

import com.qualcomm.robotcore.hardware.DcMotorSimple

@Suppress("PropertyName")
interface SwerveDriveConstants : DriveConstants {

    val LEFT_FRONT_DIRECTION: DcMotorSimple.Direction
    val LEFT_BACK_DIRECTION: DcMotorSimple.Direction
    val RIGHT_FRONT_DIRECTION: DcMotorSimple.Direction
    val RIGHT_BACK_DIRECTION: DcMotorSimple.Direction
    val LEFT_FRONT_NAME: String
    val LEFT_BACK_NAME: String
    val RIGHT_FRONT_NAME: String
    val RIGHT_BACK_NAME: String
    val LEFT_FRONT_SWIVEL_NAME: String
    val LEFT_BACK_SWIVEL_NAME: String
    val RIGHT_FRONT_SWIVEL_NAME: String
    val RIGHT_BACK_SWIVEL_NAME: String
}