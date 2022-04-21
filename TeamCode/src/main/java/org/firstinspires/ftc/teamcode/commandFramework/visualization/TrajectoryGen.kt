@file:Suppress("unused")

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.firstinspires.ftc.teamcode.commandFramework.example.trajectoryfactory.ExampleTrajectoryFactory
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.TrajectoryFactory

object TrajectoryGen {

    private val trajectoryFactory: TrajectoryFactory = ExampleTrajectoryFactory

    fun drawOffbounds() {
        GraphicsUtil.fillRect(Vector2d(0.0, -63.0), 18.0, 18.0) // robot against the wall
    }
    
    fun createTrajectory(): ArrayList<Trajectory> {
        trajectoryFactory.initialize()
        val parallelTrajectories = createParallelTrajectory()
        val trajectories = arrayListOf<Trajectory>()
        for (parallelTrajectory in parallelTrajectories) {
            trajectories.add(parallelTrajectory.trajectory)
        }
        return trajectories
    }
}