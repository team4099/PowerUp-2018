package org.usfirst.frc.team4099.auto.actions

import org.usfirst.frc.team4099.robot.subsystems.Intake

class OpenIntakeAction : Action {
    private val mIntake = Intake.instance

    override fun isFinished(): Boolean {
        return true
    }

    override fun update() {

    }

    override fun done() {

    }

    override fun start() {
        mIntake.open = true
    }
}