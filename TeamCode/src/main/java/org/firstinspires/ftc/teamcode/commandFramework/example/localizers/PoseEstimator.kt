package org.firstinspires.ftc.teamcode.commandFramework.example.localizers

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

object PoseEstimator : Localizer {
    override var poseEstimate: Pose2d
        get() = odometryPositions.last().second + absoluteAdjustment + manualAdjustment
        set(value) {
            manualAdjustment = value - poseEstimate
        }
    override val poseVelocity: Pose2d?
        get() = OdometryLocalizer.poseVelocity

    private val odometryPositions: ArrayList<Pair<Double, Pose2d>> = arrayListOf(Pair(0.0, Pose2d()))
    private var absoluteAdjustment = Pose2d()
    private var manualAdjustment = Pose2d()

    private val timer = ElapsedTime()

    fun initialize() {
        timer.reset()
    }

    override fun update() {
        odometryPositions.add(Pair(timer.milliseconds(), OdometryLocalizer.poseEstimate))
    }

    fun addAbsoluteMeasurement(position: Pose2d, delayTimeMillis: Double) {
        var nearestTime: Double? = null
        var nearestPair: Pair<Double, Pose2d>? = null
        for (pair in odometryPositions) {
            val time: Double = abs(timer.milliseconds() - delayTimeMillis - pair.first)
            if (nearestTime == null || time < nearestTime) {
                nearestPair = pair
                nearestTime = time
            }
        }
        if (nearestPair != null)
            absoluteAdjustment = position - nearestPair.second
    }
}