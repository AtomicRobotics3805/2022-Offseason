package org.firstinspires.ftc.teamcode.commandFramework.visualization

import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity
import com.noahbres.meepmeep.roadrunner.trajectorysequence.SequenceSegment
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySegment
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.CommandGroup
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.driving.DriveConstants
import org.firstinspires.ftc.teamcode.commandFramework.driving.FollowTrajectory
import org.firstinspires.ftc.teamcode.commandFramework.driving.drivers.MecanumDrive
import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.example.drive.ExampleMecanumDriveConstants
import org.firstinspires.ftc.teamcode.commandFramework.example.localizers.ExampleOdometryConstants
import org.firstinspires.ftc.teamcode.commandFramework.example.trajectoryfactory.ExampleTrajectoryFactory
import org.firstinspires.ftc.teamcode.commandFramework.sequential

fun main() {
    Constants.drive = MecanumDrive(
        ExampleMecanumDriveConstants, TwoWheelOdometryLocalizer(
            ExampleOdometryConstants()
        )
    )
    ExampleTrajectoryFactory.initialize()
    val routine: Command = sequential {
        +FollowTrajectory(ExampleTrajectoryFactory.startToHubFront)
    }
    val constants: DriveConstants = ExampleMecanumDriveConstants
    val meepMeep = MeepMeep(600)
    val botBuilder: DefaultBotBuilder = DefaultBotBuilder(meepMeep)
        .setConstraints(
            constants.MAX_VEL, constants.MAX_ACCEL,
            constants.MAX_ANG_VEL, constants.MAX_ANG_ACCEL, constants.TRACK_WIDTH
        )
    val bot: RoadRunnerBotEntity = botBuilder.followTrajectorySequence(
        TrajectorySequence(
            routineToSegmentList(routine as CommandGroup)
        )
    )
    meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)
        .start()
}

fun routineToSegmentList(routine: CommandGroup): List<SequenceSegment> {
    val trajectories = arrayListOf<SequenceSegment>()
    for (command in routine.commands) {
        if (command is FollowTrajectory) {
            trajectories.add(TrajectorySegment(command.trajectory.trajectory))
        }
        if (command is CommandGroup) {
            trajectories.addAll(routineToSegmentList(command))
        }
    }
    return trajectories
}