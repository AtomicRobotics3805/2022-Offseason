package org.firstinspires.ftc.teamcode.commandFramework

import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectory

interface Driver {

    var follower: TrajectoryFollower
    var trajectory: ParallelTrajectory?
}