

package org.firstinspires.ftc.teamcode.commandFramework.driving.drivers

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.TankDrive as RoadRunnerTankDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.kinematics.TankKinematics
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.Constants.driveConstants
import org.firstinspires.ftc.teamcode.commandFramework.TelemetryController
import org.firstinspires.ftc.teamcode.commandFramework.driving.Driver
import org.firstinspires.ftc.teamcode.commandFramework.driving.DriverControlled
import org.firstinspires.ftc.teamcode.commandFramework.driving.FollowTrajectory
import org.firstinspires.ftc.teamcode.commandFramework.driving.Turn
import org.firstinspires.ftc.teamcode.commandFramework.example.localizers.OdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectory
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectoryBuilder
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.CustomCommand
import org.firstinspires.ftc.teamcode.roadrunnerutil.roadrunner.DashboardUtil
import org.firstinspires.ftc.teamcode.roadrunnerutil.roadrunner.LynxModuleUtil
import java.util.*
import kotlin.math.abs

/**
 * This object controls the movement of a tank drivetrain. It's pretty complicated, so it might be
 * tough to understand everything at first. The normal wrapping & spacing rules aren't always
 * followed because they can make the file more difficult to understand.
 *
 * If you want to have multiple Tank Drives in the same project, you don't need to make another copy
 * of this object. You just need to make another object that inherits DriveConstants and set that as
 * the one used in the Constants file.
 */
@Config
object TankDrive : RoadRunnerTankDrive(
        driveConstants.kV,
        driveConstants.kA,
        driveConstants.kStatic,
        driveConstants.TRACK_WIDTH,
    ), Driver, Subsystem {

    // these values don't need to be changed in the Road Runner dashboard, which is why they are
    // here instead of in MecanumDriveConstants
    private const val POSE_HISTORY_LIMIT = 100
    const val VUFORIA_KEY = " AZ2jk6P/////AAABmck8NCyjWkCGvLdpx9HZ1kxI2vQPDlzN9vJnqy69nXRjvoXgBCEWZasRnd1hFjBpRiSXw4G4JwDFsk3kNSVko2UkuCgbi/RsiODF76MtldIi6YZGfrRMZTICMKwTanuOysh4Cn9Xd9nZzCpDiLAPLsUtKoj/DdBUn0gJuARMglUPW7/qirgtk0xI232ttZpXhgh9ya8R8LxnH+UTCCFtEaQft2ru0Tv+30Un82gG1uEzcrMc/8F3lefedcOTrelPQx8xUD8cME9dj99b5oZWfM60b36/xdswhYF7pygskPtXCS28j81xWKHGNhr5s8xL91cbKOovDzdJYdfVIILZnL1sjdbtN8zW4mULOYHwO4ur"

    // these two are used to follow trajectories & turn
    override val follower: HolonomicPIDVAFollower
        get() = HolonomicPIDVAFollower(
            driveConstants.TRANSLATIONAL_PID, driveConstants.TRANSLATIONAL_PID, driveConstants.HEADING_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5
        )
    override val turnController: PIDFController
        get() = PIDFController(driveConstants.HEADING_PID)

    // the heading of the robot directly from the IMU
    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()

    // these constraints are used when building trajectories to determine how fast the robot will go
    private val velConstraint: MinVelocityConstraint
        get() = MinVelocityConstraint(listOf(
            AngularVelocityConstraint(driveConstants.MAX_ANG_VEL),
            MecanumVelocityConstraint(driveConstants.MAX_VEL, driveConstants.TRACK_WIDTH))
        )
    private val accelConstraint: ProfileAccelerationConstraint
        get() = ProfileAccelerationConstraint(driveConstants.MAX_ACCEL)

    private val poseHistory = LinkedList<Pose2d>()

    // drive motors, battery voltage sensor, IMU, and the OpMode's hardwareMap
    private lateinit var leftMotor: DcMotorEx
    private lateinit var rightMotor: DcMotorEx
    private lateinit var motors: List<DcMotorEx>
    private lateinit var batteryVoltageSensor: VoltageSensor
    private lateinit var imu: BNO055IMU
    private lateinit var hardwareMap: HardwareMap

    // these two variables are used in TeleOp to control "slow mode"
    var driverSpeedIndex = 0
    override val driverSpeed: Double
        get() = driveConstants.DRIVER_SPEEDS[driverSpeedIndex]

    // this is the trajectory that the robot is currently following
    override var trajectory: ParallelTrajectory? = null

    /**
     * Switches TeleOp speeds, also known as slow mode
     */
    fun switchSpeed(): Command = CustomCommand(_start = {
            driverSpeedIndex++
            if (driverSpeedIndex >= driveConstants.DRIVER_SPEEDS.size)
                driverSpeedIndex = 0
        })
    /**
     * Allows the drivers to control the drivetrain using a gamepad
     * @param gamepad the gamepad that controls the drivetrain
     */
    fun driverControlled(gamepad: Gamepad): Command = DriverControlled(gamepad, listOf(this), true)
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
        Turn(angle, driveConstants.MAX_ACCEL, driveConstants.MAX_VEL, turnType, listOf(this), true)

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
     * Initializes the drivetrain. This includes initializing the IMU, motor, and the battery
     * voltage sensor.
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
        // initializes the motors
        leftMotor = hardwareMap.get(DcMotorEx::class.java, "leftMotor")
        rightMotor = hardwareMap.get(DcMotorEx::class.java, "rightMotor")
        motors = listOf(leftMotor, rightMotor)
        // sets the achieveableMaxRPMFraction for each motor to 1.0
        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }
        // sets the RunMode for each motor
        if (driveConstants.IS_RUN_USING_ENCODER) {
            for (motor in motors) {
                motor.mode = RunMode.STOP_AND_RESET_ENCODER
                motor.mode = RunMode.RUN_USING_ENCODER
            }
            // sets the motors' PIDFCoefficients
            setPIDFCoefficients(driveConstants.MOTOR_VEL_PID)
        }
        else {
            for (motor in motors) {
                motor.mode = RunMode.RUN_WITHOUT_ENCODER
            }
        }
        // sets the zero power behavior for each motor
        for (motor in motors) {
            motor.zeroPowerBehavior = ZeroPowerBehavior.BRAKE
        }
        // reverses motors if necessary
        leftMotor.direction = DcMotorSimple.Direction.REVERSE
        // sets the localizer
        localizer = OdometryLocalizer
    }

    /**
     * Displays the robot and its position history on the FtcDashboard
     */
    override fun periodic() {
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

    /**
     * Adjusts the drive power to make sure that no motors will receive a power of over 1
     * @param drivePower the power (with x, y, and heading params) of the drivetrain
     */
    override fun setWeightedDrivePower(drivePower: Pose2d) {
        var vel = drivePower
        val denominator = abs(drivePower.x) + abs(drivePower.y) + abs(drivePower.heading)
        if (denominator > 1) {
            vel = Pose2d(drivePower.x, drivePower.y, drivePower.heading).div(denominator)
        }
        setDrivePower(vel)
    }

    /**
     * Returns a list of how far each wheel has turned in inches
     * @return how far each wheel has turned in inches
     */
    override fun getWheelPositions(): List<Double> {
        val wheelPositions: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelPositions.add(driveConstants.encoderTicksToInches(motor.currentPosition.toDouble()))
        }
        return wheelPositions
    }

    /**
     * Returns a list of how fast each wheel is turning in inches per second
     * @return how fast each wheel is turning in inches per second
     */
    override fun getWheelVelocities(): List<Double> {
        val wheelVelocities: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelVelocities.add(driveConstants.encoderTicksToInches(motor.velocity))
        }
        return wheelVelocities
    }

    /**
     * Sets the robot's drive signal. The only reason why this is overridden is so that we can use
     * updated constants instead of the constants passed into the constructor.
     * @param driveSignal the target velocity and acceleration of the robot
     */
    override fun setDriveSignal(driveSignal: DriveSignal) {
        val velocities = TankKinematics.robotToWheelVelocities(
            driveSignal.vel,
            driveConstants.TRACK_WIDTH
        )
        val accelerations = TankKinematics.robotToWheelAccelerations(
            driveSignal.accel,
            driveConstants.TRACK_WIDTH
        )
        val powers = Kinematics.calculateMotorFeedforward(
            velocities,
            accelerations,
            driveConstants.kV,
            driveConstants.kA,
            driveConstants.kStatic
        )
        setMotorPowers(powers[0], powers[1])
    }

    /**
     * Sets power to each of the motors
     * @param left the power for the left motor
     * @param right the power for the right motor
     */
    override fun setMotorPowers(left: Double, right: Double) {
        leftMotor.power = left
        rightMotor.power = right
    }

    /**
     * Sets the PIDF coefficients for the built-in velocity control on the motors. Adjusts the f
     * value based on the battery voltage.
     * @param coefficients the PIDF coefficients to set the motors to
     */
    private fun setPIDFCoefficients(coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
            coefficients.p, coefficients.i, coefficients.d,
            coefficients.f * 12 / batteryVoltageSensor.voltage
        )
        for (motor in motors) {
            motor.setPIDFCoefficients(RunMode.RUN_USING_ENCODER, compensatedCoefficients)
        }
    }
}