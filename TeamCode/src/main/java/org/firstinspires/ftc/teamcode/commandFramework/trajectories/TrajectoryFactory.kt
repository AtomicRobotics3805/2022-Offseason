package org.firstinspires.ftc.teamcode.commandFramework.trajectories

/**
 * This class contains all of the RoadRunner trajectories and start positions in the project. It's
 * used in routines. You can find how to use each of the possible trajectory segments (like back and =
 * splineToSplineHeading) here: https://learnroadrunner.com/trajectorybuilder-functions.html
 * The coordinate system for Road Runner is as follows:
 * Positive x is up, negative x is down, positive y is right, negative y is left, positive angles
 * are counterclockwise. All measurements are in inches and radians.
 */
abstract class TrajectoryFactory {

    var initialized = false

    /**
     * Sets initialized to true and initializes any start positions and trajectories
     */
    open fun initialize() {
        initialized = true
    }
}