package org.usfirst.frc.team4099.robot.loops

import edu.wpi.first.wpilibj.PowerDistributionPanel

/** Manages the shutting off of components and subsystems when at risk of brownout.
 * It does this through a multitude of steps:
 * 1. Constantly monitor the Battery voltage
 * 2.
 * 3.
 */
class BrownoutDefender private constructor() : Loop {
    private val pdp = PowerDistributionPanel()

    override fun onStart() {
        pdp.clearStickyFaults()
    }

    override fun onLoop() {

    }

    override fun onStop() {

    }

    fun getCurrent(channel: Int): Double {
        return pdp.getCurrent(channel)
    }

    companion object {
        val instance = BrownoutDefender()
    }
}
