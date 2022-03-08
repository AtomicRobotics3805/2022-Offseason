

package org.firstinspires.ftc.teamcode.commandFramework.example.drivers

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.MecanumDrive as RoadRunnerMecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
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
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.commandFramework.Command
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.TelemetryController
import org.firstinspires.ftc.teamcode.commandFramework.driving.Driver
import org.firstinspires.ftc.teamcode.commandFramework.driving.DriverControlled
import org.firstinspires.ftc.teamcode.commandFramework.driving.FollowTrajectory
import org.firstinspires.ftc.teamcode.commandFramework.driving.Turn
import org.firstinspires.ftc.teamcode.commandFramework.example.drivers.MecanumDriveConstants as Const
import org.firstinspires.ftc.teamcode.commandFramework.example.localizers.OdometryLocalizer
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectory
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.ParallelTrajectoryBuilder
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.inchesToMm
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.toRadians
import org.firstinspires.ftc.teamcode.commandFramework.utilCommands.CustomCommand
import org.firstinspires.ftc.teamcode.roadrunnerutil.roadrunner.DashboardUtil
import org.firstinspires.ftc.teamcode.roadrunnerutil.roadrunner.LynxModuleUtil
import java.util.*
import kotlin.math.abs

/**
 * This object contains all of the constants used by MecanumDrive. The reason why the constants
 * aren't in the MecanumDrive object is because they need to be used in the MecanumDrive
 * constructor. When copying this file and making adjustments for a new robot, you should only have
 * to change these constants.
 */


@Config
object MecanumDriveConstants {

    // These are motor constants that should be listed online for your motors.
    @JvmField
    var TICKS_PER_REV = 537.7 // 5203 312 RPM Yellow Jacket
    @JvmField
    var MAX_RPM = 312.0
    
    /*
     * Set runUsingEncoder to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present or an alternative localization
     * method is in use (e.g., dead wheels).
     *
     * If using the built-in motor velocity PID, update motorVeloPID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    @JvmField
    var MOTOR_VELO_PID = PIDFCoefficients(0.0, 0.0, 0.0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV))
    @JvmField
    var IS_RUN_USING_ENCODER = false

    /*
     * If not using the built-in motor velocity PID, then these coefficients need to be tuned. They
     * control the robot's feedforward movement (its pre-planned motor movements, as opposed to
     * feedback, which adjusts motor movements mid-trajectory)
     */
    @JvmField
    var kV = 0.013
    @JvmField
    var kA = 0.0025
    @JvmField
    var kStatic = 0.01

    /*
     * These constants are tied to your robot's hardware. You should be able to find them just by
     * looking at the robot. TRACK_WIDTH may require additional tuning, however.
     */
    @JvmField
    var WHEEL_RADIUS = 2.0 // in
    @JvmField
    var GEAR_RATIO = 1.0 // output (wheel) speed / input (motor) speed
    @JvmField
    var TRACK_WIDTH = 18.0 // in, the distance between center of left and right drive wheels

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling). All distance units are inches & time in seconds. The
     * angular values are in radians.
     */
    @JvmField
    var MAX_VEL = 52.0
    @JvmField
    var MAX_ACCEL = 45.0
    @JvmField
    var MAX_ANG_VEL = 90.0.toRadians
    @JvmField
    var MAX_ANG_ACCEL = 90.0.toRadians

    /*
     * These values are used solely with Mecanum Drives to adjust the kinematics functions that
     * translate robot velocity to motor speeds. The only way to find these values is to tune and
     * adjust them until they seem about right. They should be close to 1.0.
     * The drift multipliers make the robot move forward/backward or turn left/right when strafing
     * to counteract the drift caused by an off-center center of gravity. The lateral multiplier
     * is meant to increase left/right speed.
     */
    @JvmField
    var LATERAL_MULTIPLIER = 1.0
    @JvmField
    var DRIFT_MULTIPLIER = 1.0
    @JvmField
    var DRIFT_TURN_MULTIPLIER = 1.0
    
    /*
     * These coefficients are used to adjust your location and heading when they don't match up with
     * where you should be. The only way to get these values is through tuning, but generally P=8
     * I=0 and D=0 are reasonable starting points.
     */
    @JvmField
    var TRANSLATIONAL_PID = PIDCoefficients(8.0, 0.0, 0.0)
    @JvmField
    var HEADING_PID = PIDCoefficients(8.0, 0.0, 0.0)
    
    // camera location, only necessary if you're using cameras obviously
    @JvmField
    var CAMERA_FORWARD_DISPLACEMENT = 0.0.inchesToMm.toFloat()
    @JvmField
    var CAMERA_VERTICAL_DISPLACEMENT = 0.0.inchesToMm.toFloat()
    @JvmField
    var CAMERA_LEFT_DISPLACEMENT = 0.0.inchesToMm.toFloat()

    // used during TeleOp to make precise movements
    @JvmField
    var driverSpeeds = listOf(0.1, 0.4, 1.0)

    fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }

    fun rpmToVelocity(rpm: Double): Double {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0
    }

    fun getMotorVelocityF(ticksPerSecond: Double): Double {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond
    }
}

/**
 * This object controls the movement of the drivetrain. It's probably the most complicated class
 * in the whole project, so it might be tough to understand everything at first. The normal
 * wrapping & spacing rules aren't always followed because they can make the file more difficult to
 * understand.
 */
@Config
object MecanumDrive : RoadRunnerMecanumDrive(
        Const.kV,
        Const.kA,
        Const.kStatic,
        Const.TRACK_WIDTH,
        Const.TRACK_WIDTH,
        Const.LATERAL_MULTIPLIER
    ), Driver, Subsystem {

    // these values don't need to be changed in the Road Runner dashboard, which is why they are
    // here instead of in MecanumDriveConstants
    private const val POSE_HISTORY_LIMIT = 100
    const val VUFORIA_KEY = " AZ2jk6P/////AAABmck8NCyjWkCGvLdpx9HZ1kxI2vQPDlzN9vJnqy69nXRjvoXgBCEWZasRnd1hFjBpRiSXw4G4JwDFsk3kNSVko2UkuCgbi/RsiODF76MtldIi6YZGfrRMZTICMKwTanuOysh4Cn9Xd9nZzCpDiLAPLsUtKoj/DdBUn0gJuARMglUPW7/qirgtk0xI232ttZpXhgh9ya8R8LxnH+UTCCFtEaQft2ru0Tv+30Un82gG1uEzcrMc/8F3lefedcOTrelPQx8xUD8cME9dj99b5oZWfM60b36/xdswhYF7pygskPtXCS28j81xWKHGNhr5s8xL91cbKOovDzdJYdfVIILZnL1sjdbtN8zW4mULOYHwO4ur"

    // turns the displacement constants into an OpenGLMatrix.
    val cameraLocationOnRobot: OpenGLMatrix
        get() = OpenGLMatrix
            .translation(Const.CAMERA_FORWARD_DISPLACEMENT, Const.CAMERA_LEFT_DISPLACEMENT, Const.CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES, 90f, 90f, 0f))

    // these two are used to follow trajectories & turn
    override val follower: HolonomicPIDVAFollower
        get() = HolonomicPIDVAFollower(
            Const.TRANSLATIONAL_PID, Const.TRANSLATIONAL_PID, Const.HEADING_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5
        )
    override val turnController: PIDFController
        get() = PIDFController(Const.HEADING_PID)

    // the heading of the robot directly from the IMU
    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()

    // these constraints are used when building trajectories to determine how fast the robot will go
    private val velConstraint: MinVelocityConstraint
        get() = MinVelocityConstraint(listOf(
            AngularVelocityConstraint(Const.MAX_ANG_VEL),
            MecanumVelocityConstraint(Const.MAX_VEL, Const.TRACK_WIDTH))
        )
    private val accelConstraint: ProfileAccelerationConstraint
        get() = ProfileAccelerationConstraint(Const.MAX_ACCEL)

    private val poseHistory = LinkedList<Pose2d>()

    // drive motors, battery voltage sensor, IMU, and the OpMode's hardwareMap
    private lateinit var leftFront: DcMotorEx
    private lateinit var leftRear: DcMotorEx
    private lateinit var rightRear: DcMotorEx
    private lateinit var rightFront: DcMotorEx
    private lateinit var motors: List<DcMotorEx>
    private lateinit var batteryVoltageSensor: VoltageSensor
    private lateinit var imu: BNO055IMU
    private lateinit var hardwareMap: HardwareMap

    // these two variables are used in TeleOp to control "slow mode"
    var driverSpeedIndex = 0
    override val driverSpeed: Double
        get() = Const.driverSpeeds[driverSpeedIndex]

    // this is the trajectory that the robot is currently following
    override var trajectory: ParallelTrajectory? = null

    /**
     * Switches TeleOp speeds, also known as slow mode
     */
    fun switchSpeed(): Command = CustomCommand(_start = {
            driverSpeedIndex++
            if (driverSpeedIndex >= Const.driverSpeeds.size)
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
        Turn(angle, Const.MAX_ACCEL, Const.MAX_VEL, turnType, listOf(this), true)

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
        leftFront = hardwareMap.get(DcMotorEx::class.java, "LF")
        leftRear = hardwareMap.get(DcMotorEx::class.java, "LB")
        rightRear = hardwareMap.get(DcMotorEx::class.java, "RB")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "RF")
        motors = listOf(leftFront, leftRear, rightRear, rightFront)
        // sets the achieveableMaxRPMFraction for each motor to 1.0
        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }
        // sets the RunMode for each motor
        if (Const.IS_RUN_USING_ENCODER) {
            for (motor in motors) {
                motor.mode = RunMode.STOP_AND_RESET_ENCODER
                motor.mode = RunMode.RUN_USING_ENCODER
            }
            // sets the motors' PIDFCoefficients
            setPIDFCoefficients(Const.MOTOR_VELO_PID)
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
        leftRear.direction = DcMotorSimple.Direction.REVERSE
        leftFront.direction = DcMotorSimple.Direction.REVERSE
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
            wheelPositions.add(Const.encoderTicksToInches(motor.currentPosition.toDouble()))
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
            wheelVelocities.add(Const.encoderTicksToInches(motor.velocity))
        }
        return wheelVelocities
    }

    /**
     * Sets power to each of the motors
     * @param frontLeft the power for the front left motor
     * @param rearLeft the power for the rear left motor
     * @param rearRight the power for the rear right motor
     * @param frontRight the power for the front right motor
     */
    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        leftFront.power = frontLeft
        leftRear.power = rearLeft
        rightRear.power = rearRight
        rightFront.power = frontRight
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

    override val TICKS_PER_REV: Double
        get() = Const.TICKS_PER_REV
    override val MAX_RPM: Double
        get() = Const.MAX_RPM
    override val MOTOR_VELO_PID: PIDFCoefficients
        get() = Const.MOTOR_VELO_PID
    override val IS_RUN_USING_ENCODER: Boolean
        get() = Const.IS_RUN_USING_ENCODER
    override val kV: Double
        get() = Const.kV
    override val kA: Double
        get() = Const.kA
    override val kStatic: Double
        get() = Const.kStatic
    override val WHEEL_RADIUS: Double
        get() = Const.WHEEL_RADIUS
    override val GEAR_RATIO: Double
        get() = Const.GEAR_RATIO
    override val TRACK_WIDTH: Double
        get() = Const.TRACK_WIDTH
    override val MAX_VEL: Double
        get() = Const.MAX_VEL
    override val MAX_ACCEL: Double
        get() = Const.MAX_ACCEL
    override val MAX_ANG_VEL: Double
        get() = Const.MAX_ANG_VEL
    override val MAX_ANG_ACCEL: Double
        get() = Const.MAX_ANG_ACCEL
    override val LATERAL_MULTIPLIER: Double
        get() = Const.LATERAL_MULTIPLIER
    override val DRIFT_MULTIPLIER: Double
        get() = Const.DRIFT_MULTIPLIER
    override val DRIFT_TURN_MULTIPLIER: Double
        get() = Const.DRIFT_TURN_MULTIPLIER
    override val TRANSLATION_PID: PIDCoefficients
        get() = Const.TRANSLATIONAL_PID
    override val HEADING_PID: PIDCoefficients
        get() = Const.HEADING_PID
}