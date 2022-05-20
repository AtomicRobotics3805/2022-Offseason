package org.firstinspires.ftc.teamcode.main.visualization

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.MecanumDriveWheelLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.sequential
import org.firstinspires.ftc.teamcode.commandFramework.visualization.MeepMeepVisualizer
import org.firstinspires.ftc.teamcode.main.other.TrajectoryFactory
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants

fun main() {
    MeepMeepVisualizer.addRobot(
        MecanumDrive(
            DriveConstants,
            { MecanumDriveWheelLocalizer(Constants.drive as MecanumDrive) }
        ) { Pose2d() },
        { sequential { } },
        Constants.Color.BLUE
    )
    MeepMeepVisualizer.run(TrajectoryFactory)
}