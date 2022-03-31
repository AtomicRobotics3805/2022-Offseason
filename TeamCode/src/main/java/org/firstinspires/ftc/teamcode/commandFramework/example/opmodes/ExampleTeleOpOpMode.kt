package org.firstinspires.ftc.teamcode.commandFramework.example.opmodes

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.example.controls.ExampleControls
import org.firstinspires.ftc.teamcode.commandFramework.example.drive.ExampleMecanumDriveConstants
import org.firstinspires.ftc.teamcode.commandFramework.example.localizers.ExampleOdometryConstants
import org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms.Claw
import org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms.Lift
import org.firstinspires.ftc.teamcode.commandFramework.example.routines.ExampleRoutines
import org.firstinspires.ftc.teamcode.commandFramework.example.trajectoryfactory.ExampleTrajectoryFactory
import org.firstinspires.ftc.teamcode.commandFramework.opmodes.TeleOpOpMode

@Disabled
@TeleOp(name = "Example Auto OpMode")
class ExampleTeleOpOpMode : TeleOpOpMode(
    ExampleControls,
    Constants.Color.UNKNOWN,
    ExampleTrajectoryFactory,
    { ExampleRoutines.initializationRoutine },
    null,
    MecanumDrive(
        ExampleMecanumDriveConstants,
        TwoWheelOdometryLocalizer(ExampleOdometryConstants()),
        Constants.endPose ?: Pose2d()
    ),
    Lift,
    Claw
)