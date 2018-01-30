package org.usfirst.frc.team4099.robot.loops

/** Manages the shutting off of components and subsystems when at risk of brownout.
 * It does this through a multitude of steps:
 * 1. Constantly monitor the Battery voltage
 * 2.
 * 3.
 */
class BrownoutDefender private constructor() : Loop {

    override fun onStart(timestamp: Double) {

    }

    override fun onLoop(timestamp: Double) {

    }

    override fun onStop(timestamp: Double) {

    }

    companion object {
        val instance = BrownoutDefender()
    }
}
