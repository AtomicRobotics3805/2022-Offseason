package org.firstinspires.ftc.teamcode.commandFramework.driving

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectory

/**
 * This interface contains a follower and a trajectory being followed. It's used by commands like
 * DisplacementDelay. Drivetrain objects/classes like MecanumDrive should implement this interface.
 */
@Suppress("PropertyName")
interface Driver {
    var trajectory: ParallelTrajectory?
    var poseEstimate: Pose2d
    val turnController: PIDFController
    val follower: HolonomicPIDVAFollower
    val rawExternalHeading: Double
    val driverSpeed: Double
    fun setDriveSignal(driveSignal: DriveSignal)
    fun setWeightedDrivePower(drivePower: Pose2d)
    fun getExternalHeadingVelocity(): Double?

    val TICKS_PER_REV: Double
    val MAX_RPM: Double
    val MOTOR_VELO_PID: PIDFCoefficients
    val IS_RUN_USING_ENCODER: Boolean
    val kV: Double
    val kA: Double
    val kStatic: Double
    val WHEEL_RADIUS: Double
    val GEAR_RATIO: Double
    val TRACK_WIDTH: Double
    val MAX_VEL: Double
    val MAX_ACCEL: Double
    val MAX_ANG_VEL: Double
    val MAX_ANG_ACCEL: Double
    val LATERAL_MULTIPLIER: Double
    val DRIFT_MULTIPLIER: Double
    val DRIFT_TURN_MULTIPLIER: Double
    val TRANSLATION_PID: PIDCoefficients
    val HEADING_PID: PIDCoefficients
}