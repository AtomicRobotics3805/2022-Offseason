package org.firstinspires.ftc.teamcode.main.other

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.TrajectoryFactory

/**
 * This object controls RoadRunner trajectories. These trajectories should be created and tested
 * in the RRPathVisualizer project, then copied over to here. It also manages start positions for
 * the robot, since those are inherently tied to the trajectories. It's empty since this is just a
 * template project.
 */
@Suppress("unused", "MemberVisibilityCanBePrivate")
@Config
object TrajectoryFactory : TrajectoryFactory() {

    /**
     * Initializes the start positions and the trajectories. That includes setting the actual
     * coordinates and path segments. This should all be done in RRPathVisualizer first, then copied
     * over here.
     */
    override fun initialize() {

    }
}