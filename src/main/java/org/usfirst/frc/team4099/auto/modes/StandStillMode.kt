package org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.auto.AutoModeEndedException

/**
 * Fallback for when all autonomous modes do not work, resulting in a robot
 * standstill
 */
class StandStillMode : AutoModeBase() {

    @Throws(AutoModeEndedException::class)
    override fun routine() {
        println("Starting Stand Still Mode... Done!")
    }
}
