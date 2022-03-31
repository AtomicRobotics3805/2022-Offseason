package org.firstinspires.ftc.teamcode.commandFramework.example.routines

import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants.drive
import org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms.Claw
import org.firstinspires.ftc.teamcode.commandFramework.example.mechanisms.Lift
import org.firstinspires.ftc.teamcode.commandFramework.example.trajectoryfactory.ExampleTrajectoryFactory
import org.firstinspires.ftc.teamcode.commandFramework.parallel
import org.firstinspires.ftc.teamcode.commandFramework.sequential

object ExampleRoutines {

    val initializationRoutine: Command
        get() = parallel {
            +Lift.toLow
            +Claw.close
        }

    val mainRoutine: Command
        get() = sequential {
            +drive.followTrajectory(ExampleTrajectoryFactory.startToHubFront)
            +Lift.toHigh
            +Claw.open
            +parallel {
                +Lift.toLow
                +drive.followTrajectory(ExampleTrajectoryFactory.hubFrontToPark)
            }
        }
}