package org.usfirst.frc.team4099.auto.actions

import org.usfirst.frc.team4099.robot.subsystems.Intake

class CloseIntakeAction : Action {
    private val intake = Intake.instance

    override fun update() { }

    override fun isFinished(): Boolean {
        return true
    }

    override fun done() { }

    override fun start() {
        intake.open = false
    }

}