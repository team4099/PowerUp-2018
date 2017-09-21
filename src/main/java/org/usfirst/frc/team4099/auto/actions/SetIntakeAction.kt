package org.usfirst.frc.team4099.auto.actions

import org.usfirst.frc.team4099.robot.subsystems.Intake

/**
 * Created by plato2000 on 2/14/17.
 */
class SetIntakeAction(private val positionToSet: Intake.IntakePosition) : Action {
    private val mIntake: Intake = Intake.instance
    private var isDone: Boolean = false

    override fun isFinished(): Boolean {
        return isDone
    }

    override fun update() {

    }

    override fun done() {

    }

    override fun start() {
        mIntake.updateIntake(positionToSet)
        isDone = true
    }
}
