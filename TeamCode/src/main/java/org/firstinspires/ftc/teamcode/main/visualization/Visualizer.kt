package org.firstinspires.ftc.teamcode.main.visualization

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.sequential
import org.firstinspires.ftc.teamcode.commandFramework.visualization.MeepMeepVisualizer
import org.firstinspires.ftc.teamcode.main.other.TrajectoryFactory
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants
import org.firstinspires.ftc.teamcode.main.subsystems.drive.OdometryConstants

fun main() {
    MeepMeepVisualizer.addRobot(
        MecanumDrive(
            DriveConstants,
            TwoWheelOdometryLocalizer(OdometryConstants)
        ) { Pose2d() },
        sequential { },
        Constants.Color.BLUE
    )
    MeepMeepVisualizer.run(TrajectoryFactory)
}