package org.firstinspires.ftc.teamcode.commandFramework.trajectories

abstract class TrajectoryFactory {

    var initialized = false

    open fun initialize() {
        initialized = true
    }
}