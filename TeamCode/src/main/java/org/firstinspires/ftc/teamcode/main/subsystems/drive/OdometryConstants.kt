package org.firstinspires.ftc.teamcode.main.subsystems.drive

import org.firstinspires.ftc.teamcode.commandFramework.driving.localizers.OdometryConstants

@Suppress("ObjectPropertyName")
object OdometryConstants : OdometryConstants {
    @JvmField
    var _PARALLEL_X = 0.0 // in; forward offset of the parallel wheel
    @JvmField
    var _PARALLEL_Y = 0.0 // in; left offset of the parallel wheel
    @JvmField
    var _PERPENDICULAR_X = 0.0 // in; forward offset of the perpendicular wheel
    @JvmField
    var _PERPENDICULAR_Y = 0.0 // in; left offset of the perpendicular wheel

    override val PARALLEL_X: Double
        get() = _PARALLEL_X
    override val PARALLEL_Y: Double
        get() = _PARALLEL_Y
    override val PERPENDICULAR_X: Double
        get() = _PERPENDICULAR_X
    override val PERPENDICULAR_Y: Double
        get() = _PERPENDICULAR_Y
}