package org.usfirst.frc.team4099.auto.actions

import org.usfirst.frc.team4099.robot.subsystems.Wrist

class DropWristAction : Action {
    private val mWrist = Wrist.instance

    override fun update() {
        mWrist.setOpenLoop(-.5)
    }

    override fun isFinished() : Boolean {
        return mWrist.getWristPosition() < Math.PI / 4

    }

    override fun done() {
        mWrist.setOpenLoop(0.0)
    }

    override fun start() {

    }
}