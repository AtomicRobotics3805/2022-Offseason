@file:Suppress("PropertyName")

package org.firstinspires.ftc.teamcode.commandFramework.example.localizers

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.commandFramework.Constants
import org.firstinspires.ftc.teamcode.commandFramework.subsystems.Localizer
import org.firstinspires.ftc.teamcode.roadrunnerutil.roadrunner.Encoder

@JvmField
var PARALLEL_X = 0.0 // in; forward offset of the parallel wheel
@JvmField
var PARALLEL_Y = 0.0 // in; left offset of the parallel wheel
@JvmField
var PERPENDICULAR_X = 0.0 // in; forward offset of the perpendicular wheel
@JvmField
var PERPENDICULAR_Y = 0.0 // in; left offset of the perpendicular wheel

/**
 * Determines position relative to previous position based on odometry wheels. Odometry wheels are
 * wheels that aren't attached to motors but are attached to encoders that can determine how much
 * the wheels have rotated. Based on how much they've rotated, we can determine the robot's
 * position. The reason why there are four constants assigned outside of the class is because we
 * need them to be outside of the class to use them in the constructor.
 */
@Config
object OdometryLocalizer : TwoTrackingWheelLocalizer(listOf(
    Pose2d(PARALLEL_X, PARALLEL_Y, 0.0),
    Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90.0))
)), Localizer {

    @JvmField
    var TICKS_PER_REV = 8192 // REV through bore encoder
    @JvmField
    var WHEEL_RADIUS = 0.688975 // rotacaster wheels // in
    @JvmField
    var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
    @JvmField
    var PARALLEL_REVERSED = true
    @JvmField
    var PERPENDICULAR_REVERSED = true
    @JvmField
    var X_MULTIPLIER = 1.0
    @JvmField
    var Y_MULTIPLIER = 1.0

    lateinit var perpendicularEncoder: Encoder
    lateinit var parallelEncoder: Encoder

    /**
     * Initializes the encoders & sets their direction
     */
    fun initialize() {
        perpendicularEncoder = Encoder(Constants.opMode.hardwareMap.get(DcMotorEx::class.java, "RF"))
        parallelEncoder = Encoder(Constants.opMode.hardwareMap.get(DcMotorEx::class.java, "LF"))

        if (PERPENDICULAR_REVERSED) perpendicularEncoder.direction = Encoder.Direction.REVERSE
        if (PARALLEL_REVERSED) parallelEncoder.direction = Encoder.Direction.REVERSE
    }

    /**
     * Returns the drive heading since this localizer only uses two wheels
     * @return the heading in radians
     */
    override fun getHeading(): Double {
        return Constants.drive.rawExternalHeading
    }

    /**
     * Returns the drive heading velocity since this localizer only uses two wheels
     * @return the radian/sec heading velocity
     */
    override fun getHeadingVelocity(): Double? {
        return Constants.drive.getExternalHeadingVelocity()
    }

    /**
     * Returns how many inches each wheel has turned, multiplied by X_MULTIPLIER & Y_MULTIPLIER
     * @return list of how much each wheel has turned in inches
     */
    override fun getWheelPositions(): List<Double> {
        return listOf(
            encoderTicksToInches(parallelEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
            encoderTicksToInches(perpendicularEncoder.currentPosition.toDouble()) * Y_MULTIPLIER
        )
    }

    /**
     * Returns the inch/sec velocity of each wheel, multiplied by X_MULTIPLIER & Y_MULTIPLIER
     * @return list of inch/sec wheel velocities
     */
    override fun getWheelVelocities(): List<Double> {
        // If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return listOf(
            encoderTicksToInches(perpendicularEncoder.rawVelocity) * X_MULTIPLIER,
            encoderTicksToInches(parallelEncoder.rawVelocity) * Y_MULTIPLIER
        )
    }

    /**
     * Translates encoder ticks to inches. Remember that each pulse gets counted as four ticks.
     * @param ticks the encoder ticks to translate
     * @return the number of inches
     */
    private fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }
}