package org.firstinspires.ftc.teamcode.commandFramework.trajectories

import org.firstinspires.ftc.teamcode.commandFramework.Constants.Color.BLUE
import org.firstinspires.ftc.teamcode.commandFramework.Constants.color

val Double.toRadians get() = (Math.toRadians(this))
val Double.switchColorAngle get () = (if (color == BLUE) this else 360 - this)
val Double.switchColor get () = (if (color == BLUE) this else this * -1)