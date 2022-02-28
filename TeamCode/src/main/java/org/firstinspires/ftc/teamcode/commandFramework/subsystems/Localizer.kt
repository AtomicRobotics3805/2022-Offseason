package org.firstinspires.ftc.teamcode.commandFramework.subsystems

import com.acmerobotics.roadrunner.localization.Localizer

interface Localizer : Localizer, Subsystem {

    override fun periodic() { update() }
}