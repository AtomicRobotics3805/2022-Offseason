package org.firstinspires.ftc.teamcode.main.testing.tuning.drivetrain

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants

@TeleOp(name = "Drivetrain Test")
class DrivetrainTest : LinearOpMode() {

    override fun runOpMode() {
        Constants.opMode = this
        CommandScheduler.registerSubsystems(Constants.drive)
        val testTrajectory = Constants.drive.trajectoryBuilder(Pose2d(), 0.0)
            .forward(20.0)
            .back(20.0)
            .build()
        waitForStart()
        CommandScheduler.scheduleCommand(Constants.drive.followTrajectory(testTrajectory))
        while(opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}