package org.firstinspires.ftc.teamcode.commandFramework.example.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.example.drive.ExampleMecanumDriveConstants
import org.firstinspires.ftc.teamcode.commandFramework.example.localizers.ExampleOdometryConstants
import org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms.Claw
import org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms.Lift
import org.firstinspires.ftc.teamcode.commandFramework.example.routines.ExampleRoutines
import org.firstinspires.ftc.teamcode.commandFramework.example.trajectoryfactory.ExampleTrajectoryFactory
import org.firstinspires.ftc.teamcode.commandFramework.opmodes.AutonomousOpMode
import org.firstinspires.ftc.teamcode.commandFramework.sequential

@Disabled
@Autonomous(name = "Example Auto OpMode")
class ExampleAutonomousOpMode : AutonomousOpMode(
    Constants.Color.BLUE,
    ExampleTrajectoryFactory,
    { ExampleRoutines.mainRoutine },
    { ExampleRoutines.initializationRoutine },
    MecanumDrive(
        ExampleMecanumDriveConstants,
        TwoWheelOdometryLocalizer(ExampleOdometryConstants()),
        Pose2d()
    ),
    Lift,
    Claw
)