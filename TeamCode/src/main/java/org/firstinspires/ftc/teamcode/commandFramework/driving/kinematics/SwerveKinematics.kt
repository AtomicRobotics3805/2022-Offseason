package org.firstinspires.ftc.teamcode.commandFramework.driving.kinematics

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.noahbres.meepmeep.core.toDegrees
import kotlin.math.*

enum class ServoType(val rotationDegrees: Double) {
    REGULAR_GOBILDA(300.0),
    GOBILDA_FIVE_TURN(1800.0)
}

/**
 * LF, LB, RB, RF
 */
fun robotToWheelVelocities(
    robotVel: Pose2d,
    servoPosition: Double,
    servoType: ServoType
): Pair<List<Double>, Double> {
    var reversed = false
    var currentDirection = servoPosition * servoType.rotationDegrees
    var targetDirection = atan(robotVel.y / (if (robotVel.x != 0.0) robotVel.x else 0.000001)).toDegrees()
    while (currentDirection > 360) {
        currentDirection -= 360
    }
    if (targetDirection < 0) {
        targetDirection += 360
    }
    var directionAdjustment = targetDirection - currentDirection
    if (abs(directionAdjustment) > 180) {
        reversed = true
        directionAdjustment += sign(directionAdjustment) * 180.0
        targetDirection -= 180.0
    }
    val wheelSpeed = sqrt(robotVel.x.pow(2) + robotVel.y.pow(2)) * if (reversed) -1 else 1
    val LFHeadingSpeed = when (targetDirection) {
        in 0.0..90.0 -> robotVel.heading
        in 90.0..180.0 -> -robotVel.heading * ((targetDirection - 135.0) / 45.0)
        in 180.0..270.0 -> -robotVel.heading
        else -> robotVel.heading * ((targetDirection-315.0) / 45.0)
    }
    val RBHeadingSpeed = -LFHeadingSpeed
    val LBHeadingSpeed = when (targetDirection) {
        in 0.0..90.0 -> -robotVel.heading * ((targetDirection - 45.0) / 45.0)
        in 90.0..180.0 -> -robotVel.heading
        in 180.0..270.0 -> robotVel.heading * ((targetDirection - 215.0) / 45.0)
        else -> robotVel.heading
    }
    val RFHeadingSpeed = -LBHeadingSpeed
    return Pair(
        listOf(
            wheelSpeed + LFHeadingSpeed,
            wheelSpeed + LBHeadingSpeed,
            wheelSpeed + RBHeadingSpeed,
            wheelSpeed + RFHeadingSpeed
        ),
        servoPosition + directionAdjustment / servoType.rotationDegrees
    )
}
