package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer
import org.usfirst.frc.team4099.robot.subsystems.Wrist

class DropWristAction : Action {
    private val mWrist = Wrist.instance
    private var time = 0.0

    override fun update() {
        mWrist.setOpenLoop(.3)
    }

    override fun isFinished() : Boolean {
        return Timer.getFPGATimestamp() - time > 0.5 // || mWrist.getWristPosition() < Math.PI / 3

    }

    override fun done() {
        mWrist.setOpenLoop(0.0)
    }

    override fun start() {
        time = Timer.getFPGATimestamp()
    }
}