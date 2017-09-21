package org.usfirst.frc.team4099.robot.loops

import org.usfirst.frc.team4099.robot.subsystems.Climber
import org.usfirst.frc.team4099.robot.subsystems.Intake

/** Manages the shutting off of components and subsystems when at risk of brownout.
 * It does this through a multitude of steps:
 * 1. Constantly monitor the Battery voltage
 * 2.
 * 3.
 */
class BrownoutDefender private constructor() : Loop {

    private val mClimber = Climber.instance
    private val mIntake = Intake.instance

    override fun onStart() {

    }

    override fun onLoop() {
        if (mClimber.climberState == Climber.ClimberState.CLIMBING) {
            if (mIntake.compressor.closedLoopControl) {
                mIntake.stopCompressor()
            }
        } else {
            if (!mIntake.compressor.closedLoopControl) {
                mIntake.startCompressor()
            }
        }
    }

    override fun onStop() {

    }

    companion object {

        val instance = BrownoutDefender()
    }
}
