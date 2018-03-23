package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer
import org.usfirst.frc.team4099.robot.subsystems.Intake

class PushCubeOutAction : Action {
    private val mIntake = Intake.instance
    private var startTime = 0.0

    override fun update() {
        mIntake.intakeState = Intake.IntakeState.OUT
    }

    override fun isFinished(): Boolean {
        return Timer.getFPGATimestamp() - startTime > 4
    }

    override fun done() {
        mIntake.intakeState = Intake.IntakeState.SLOW
    }

    override fun start() {
        startTime = Timer.getFPGATimestamp()
    }
}