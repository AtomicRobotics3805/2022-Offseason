package org.firstinspires.ftc.teamcode.main.opmodes

import org.firstinspires.ftc.teamcode.main.other.Controls
import org.firstinspires.ftc.teamcode.main.subsystems.drive.TrioDriveConstants
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.example.localizers.ExampleOdometryConstants
import org.firstinspires.ftc.teamcode.commandFramework.example.routines.ExampleRoutines
import org.firstinspires.ftc.teamcode.commandFramework.example.trajectoryfactory.ExampleTrajectoryFactory
import org.firstinspires.ftc.teamcode.commandFramework.opmodes.TeleOpMode
import org.firstinspires.ftc.teamcode.main.mechanisms.*

@Suppress("unused")
@TeleOp(name = "Trio Teleop Command System Testing")
class TrioTeleOp : TeleOpMode(
    Controls,
    Constants.Colors.UNKNOWN,
    ExampleTrajectoryFactory,
    { ExampleRoutines.initializationRoutine },
    null,
    MecanumDrive(
        TrioDriveConstants,
        TwoWheelOdometryLocalizer(ExampleOdometryConstants()),
        Constants.endPose ?: Pose2d()
    ),
    Bucket,
    BucketLock,
    Carousel,
    ContainerSensor,
    Intake,
    Lift
)