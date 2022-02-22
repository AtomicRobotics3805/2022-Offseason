package org.firstinspires.ftc.teamcode.commandFramework

import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectory

/**
 * This interface contains a follower and a trajectory being followed. It's used by commands like
 * DisplacementDelay. Drivetrain objects/classes like MecanumDrive should implement this interface.
 */
interface Driver {

    var follower: TrajectoryFollower
    var trajectory: ParallelTrajectory?
}