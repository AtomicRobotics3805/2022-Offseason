package org.firstinspires.ftc.teamcode.main.testing.tuning.drivetrain

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.commandFramework.driving.DriveConstants
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.inchesToMm
import org.firstinspires.ftc.teamcode.commandFramework.trajectories.toRadians

@Suppress("ObjectPropertyName")
object TestDriveConstants : DriveConstants {

    // These are motor constants that should be listed online for your motors.
    @JvmField
    var _TICKS_PER_REV = 537.7 // 5203 312 RPM Yellow Jacket
    @JvmField
    var _MAX_RPM = 312.0

    /*
     * Set runUsingEncoder to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present or an alternative localization
     * method is in use (e.g., dead wheels).
     *
     * If using the built-in motor velocity PID, update motorVeloPID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    @JvmField
    var _MOTOR_VEL_PID = PIDFCoefficients(0.0, 0.0, 0.0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV))
    @JvmField
    var _IS_RUN_USING_ENCODER = false

    /*
     * If not using the built-in motor velocity PID, then these coefficients need to be tuned. They
     * control the robot's feedforward movement (its pre-planned motor movements, as opposed to
     * feedback, which adjusts motor movements mid-trajectory)
     */
    @JvmField
    var _kV = 0.013
    @JvmField
    var _kA = 0.0025
    @JvmField
    var _kStatic = 0.01

    /*
     * These constants are tied to your robot's hardware. You should be able to find them just by
     * looking at the robot. TRACK_WIDTH may require additional tuning, however.
     */
    @JvmField
    var _WHEEL_RADIUS = 2.0 // in
    @JvmField
    var _GEAR_RATIO = 1.0 // output (wheel) speed / input (motor) speed
    @JvmField
    var _TRACK_WIDTH = 18.0 // in, the distance between center of left and right drive wheels

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
    var _MAX_VEL = 30.0
    @JvmField
    var _MAX_ACCEL = 30.0
    @JvmField
    var _MAX_ANG_VEL = 60.0.toRadians
    @JvmField
    var _MAX_ANG_ACCEL = 60.0.toRadians

    /*
     * These values are used solely with Mecanum Drives to adjust the kinematics functions that
     * translate robot velocity to motor speeds. The only way to find these values is to tune and
     * adjust them until they seem about right. They should be close to 1.0.
     * The drift multipliers make the robot move forward/backward or turn left/right when strafing
     * to counteract the drift caused by an off-center center of gravity. The lateral multiplier
     * is meant to increase left/right speed.
     */
    @JvmField
    var _LATERAL_MULTIPLIER = 1.0
    @JvmField
    var _DRIFT_MULTIPLIER = 1.0
    @JvmField
    var _DRIFT_TURN_MULTIPLIER = 1.0

    /*
     * These coefficients are used to adjust your location and heading when they don't match up with
     * where you should be. The only way to get these values is through tuning, but generally P=8
     * I=0 and D=0 are reasonable starting points.
     */
    @JvmField
    var _TRANSLATIONAL_PID = PIDCoefficients(8.0, 0.0, 0.0)
    @JvmField
    var _HEADING_PID = PIDCoefficients(8.0, 0.0, 0.0)

    // camera location, only necessary if you're using cameras obviously
    @JvmField
    var _CAMERA_FORWARD_DISPLACEMENT = 0.0.inchesToMm.toFloat()
    @JvmField
    var _CAMERA_VERTICAL_DISPLACEMENT = 0.0.inchesToMm.toFloat()
    @JvmField
    var _CAMERA_LEFT_DISPLACEMENT = 0.0.inchesToMm.toFloat()

    // used during TeleOp to make precise movements
    @JvmField
    var _DRIVER_SPEEDS = listOf(0.1, 0.4, 1.0)

    override val TICKS_PER_REV: Double
        get() = _TICKS_PER_REV
    override val MAX_RPM: Double
        get() = _MAX_RPM
    override val MOTOR_VEL_PID: PIDFCoefficients
        get() = _MOTOR_VEL_PID
    override val IS_RUN_USING_ENCODER: Boolean
        get() = _IS_RUN_USING_ENCODER
    override val kV: Double
        get() = _kV
    override val kA: Double
        get() = _kA
    override val kStatic: Double
        get() = _kStatic
    override val WHEEL_RADIUS: Double
        get() = _WHEEL_RADIUS
    override val GEAR_RATIO: Double
        get() = _GEAR_RATIO
    override val TRACK_WIDTH: Double
        get() = _TRACK_WIDTH
    override val MAX_VEL: Double
        get() = _MAX_VEL
    override val MAX_ACCEL: Double
        get() = _MAX_ACCEL
    override val MAX_ANG_VEL: Double
        get() = _MAX_ANG_VEL
    override val MAX_ANG_ACCEL: Double
        get() = _MAX_ANG_ACCEL
    override val LATERAL_MULTIPLIER: Double
        get() = _LATERAL_MULTIPLIER
    override val DRIFT_MULTIPLIER: Double
        get() = _DRIFT_MULTIPLIER
    override val DRIFT_TURN_MULTIPLIER: Double
        get() = _DRIFT_TURN_MULTIPLIER
    override val TRANSLATIONAL_PID: PIDCoefficients
        get() = _TRANSLATIONAL_PID
    override val HEADING_PID: PIDCoefficients
        get() = _HEADING_PID
    override val DRIVER_SPEEDS: List<Double>
        get() = _DRIVER_SPEEDS
}