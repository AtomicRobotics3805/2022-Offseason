package org.firstinspires.ftc.teamcode.commandFramework.driving.drivers

import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.Drive
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.TelemetryController
import org.firstinspires.ftc.teamcode.commandFramework.driving.FollowTrajectory
import org.firstinspires.ftc.teamcode.commandFramework.driving.Turn
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectory
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectoryBuilder
import org.firstinspires.ftc.teamcode.roadrunnerutil.roadrunner.DashboardUtil
import org.firstinspires.ftc.teamcode.roadrunnerutil.roadrunner.LynxModuleUtil
import java.util.*

/**
 * This interface contains a follower and a trajectory being followed. It's used by commands like
 * DisplacementDelay. Drivetrain objects/classes like MecanumDrive should implement this interface.
 */
@Suppress("PropertyName", "MemberVisibilityCanBePrivate", "unused")
abstract class Driver : Subsystem {

    protected val POSE_HISTORY_LIMIT = 100

    // these two are used to follow trajectories & turn
    val follower: HolonomicPIDVAFollower = HolonomicPIDVAFollower(
            Constants.driveConstants.TRANSLATIONAL_PID, Constants.driveConstants.TRANSLATIONAL_PID, Constants.driveConstants.HEADING_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5
        )
    val turnController: PIDFController = PIDFController(Constants.driveConstants.HEADING_PID)

    protected val accelConstraint: ProfileAccelerationConstraint
        get() = ProfileAccelerationConstraint(Constants.driveConstants.MAX_ACCEL)
    // different for different drive types, which is why it's abstract unlike accelConstraint
    protected abstract val velConstraint: MinVelocityConstraint

    abstract val rawExternalHeading: Double
    abstract val externalHeadingVelocity: Double

    // these two variables are used in TeleOp to control "slow mode"
    var driverSpeedIndex = 0
    val driverSpeed: Double
        get() = Constants.driveConstants.DRIVER_SPEEDS[driverSpeedIndex]

    // keeps a list of positions to upload to the FTC Dashboard
    protected val poseHistory = LinkedList<Pose2d>()
    // this is the trajectory that the robot is currently following
    var trajectory: ParallelTrajectory? = null

    lateinit var localizer: Localizer
    lateinit var poseEstimate: Pose2d
    val poseVelocity: Pose2d?
        get() = localizer.poseVelocity

    protected lateinit var hardwareMap: HardwareMap
    // I wonder what this does
    protected lateinit var batteryVoltageSensor: VoltageSensor
    // the IMU has a variety of purposes, but we use it to figure out the robot's heading (angle)
    protected lateinit var imu: BNO055IMU

    /**
     * Sets the robot's drive power as a Pose2d, which has x, y, and heading components.
     * @param drivePower the target forwards/backwards, left/right, and turn speeds for the robot
     */
    abstract fun setDrivePower(drivePower: Pose2d)
    /**
     * Sets the robot's drive signal. This drive signal includes a target velocity & acceleration.
     * @param driveSignal the target velocity and acceleration of the robot
     */
    abstract fun setDriveSignal(driveSignal: DriveSignal)
    /**
     * Allows the drivers to control the drivetrain using a gamepad
     * @param gamepad the gamepad that controls the drivetrain
     */
    abstract fun driverControlled(gamepad: Gamepad): Command

    /**
     * Drives the robot along a pre-built trajectory
     * @param trajectory the trajectory to follow, use trajectoryBuilder() to get this
     */
    fun followTrajectory(trajectory: ParallelTrajectory): Command =
        FollowTrajectory(trajectory, listOf(this), true)
    /**
     * Turns the robot either to a set angle or to an angle relative to its current
     * @param angle the angle to turn to
     * @param turnType whether the turn should be relative to current position or relative to field
     */
    fun turn(angle: Double, turnType: Turn.TurnType): Command =
        Turn(angle, Constants.driveConstants.MAX_ACCEL, Constants.driveConstants.MAX_VEL, turnType, listOf(this), true)

    /**
     * Returns a TrajectoryBuilder with a certain start position
     * @param startPose the position of the robot on the field at the start of the trajectory
     * @param reversed whether the robot should start the trajectory going in reverse
     */
    fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean = false) =
        ParallelTrajectoryBuilder(TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint))
    /**
     * Overloaded function, returns a TrajectoryBuilder with a certain start position & heading
     * @param startPose the position of the robot on the field at the start of the trajectory
     * @param startHeading the absolute direction that the robot should be driving at the start of
     *                     the trajectory (currentHeading - 180.0.toRadians is the same as reversed)
     */
    fun trajectoryBuilder(startPose: Pose2d, startHeading: Double) =
        ParallelTrajectoryBuilder(TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint))

    /**
     * Initializes everything that's common across all drivetrains: the hardwareMap, IMU, battery
     * voltage sensor and turnController.
     */
    override fun initialize() {
        hardwareMap = Constants.opMode.hardwareMap
        // initializes the imu
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        // if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        //BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        // initializes the batteryVoltageSensor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()
        // makes sure the turnController knows to go from 0 to 2pi (a full rotation in radians)
        turnController.setInputBounds(0.0, 2 * Math.PI)
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)
        // honestly I don't know what this does
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }

    /**
     * Displays the robot and its position history on the FtcDashboard
     */
    override fun periodic() {
        localizer.update()
        poseEstimate = localizer.poseEstimate
        poseHistory.add(poseEstimate)
        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst()
        }
        val fieldOverlay = TelemetryController.packet.fieldOverlay()
        TelemetryController.telemetry.addData("x", poseEstimate.x)
        TelemetryController.telemetry.addData("y", poseEstimate.y)
        TelemetryController.telemetry.addData("heading", poseEstimate.heading)
        TelemetryController.telemetry.update()
        fieldOverlay.setStroke("#3F51B5")
        DashboardUtil.drawRobot(fieldOverlay, poseEstimate)
        DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory)
    }
}