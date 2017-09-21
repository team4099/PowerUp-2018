package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer

/**
 * Action to wait for a given amount of time To use this Action, call
 * runAction(new WaitAction(your_time))
 */
class WaitAction(private val mTimeToWait: Double) : Action {
    private var mStartTime: Double = 0.toDouble()

    override fun isFinished(): Boolean {
        return Timer.getFPGATimestamp() - mStartTime >= mTimeToWait
    }

    override fun update() {

    }

    override fun done() {

    }

    override fun start() {
        mStartTime = Timer.getFPGATimestamp()
    }
}
