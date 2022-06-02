package org.firstinspires.ftc.teamcode.main.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.commandFramework.CommandScheduler
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.TelemetryController
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.PoseEstimator
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.VuforiaLocalizer
import org.firstinspires.ftc.teamcode.main.subsystems.drive.DriveConstants
import org.firstinspires.ftc.teamcode.main.subsystems.drive.OdometryConstants

@TeleOp
class LocalizationTesting : LinearOpMode() {

    override fun runOpMode() {
        // setting constants
        Constants.opMode = this
        Constants.color = Constants.Color.BLUE
        Constants.drive = MecanumDrive(DriveConstants, TwoWheelOdometryLocalizer(OdometryConstants))

        CommandScheduler.registerSubsystems(Constants.drive)
        telemetry.addData("Status", "Initialized")
        telemetry.update()
        waitForStart()
        while (true);
        while (opModeIsActive()) {
            CommandScheduler.run()
            telemetry.addData("Status", "Running")
            telemetry.update()
        }
    }
}