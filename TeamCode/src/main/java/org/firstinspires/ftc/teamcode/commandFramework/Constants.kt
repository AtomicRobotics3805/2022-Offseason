package org.firstinspires.ftc.teamcode.commandFramework

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.Driver
import org.firstinspires.ftc.teamcode.commandFramework.driving.DriveConstants
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.example.drive.ExampleDriveConstants
import org.firstinspires.ftc.teamcode.main.testing.tuning.drivetrain.TestDriveConstants

/**
 * This object contains various "constant" values used by other classes. This object is not strictly
 * necessary, but without it, we'd have to pass around the same few variables to countless different
 * places. The word "constant" is used lightly because the variables in this class are not constants
 * in the strict sense. They can change between OpModes, but stay the same during each OpMode. The
 * first thing that an OpMode should do is assign values to these variables. Otherwise, many classes
 * will break.
 */
object Constants {

    enum class Color {
        BLUE,
        RED
    }
    lateinit var color: Color
    lateinit var opMode: LinearOpMode
    var driveConstants: DriveConstants = TestDriveConstants
    var drive: Driver = MecanumDrive()
}