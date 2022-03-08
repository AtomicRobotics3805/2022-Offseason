package org.firstinspires.ftc.teamcode.commandFramework.driving

import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
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
}