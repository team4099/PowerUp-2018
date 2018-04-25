package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer
import org.usfirst.frc.team4099.robot.subsystems.Intake

class PushCubeOutAction : Action {
    private val intake = Intake.instance
    private var startTime = 0.0

    override fun update() {
        intake.intakeState = Intake.IntakeState.SLOW_OUT
    }

    override fun isFinished(): Boolean {
        return Timer.getFPGATimestamp() - startTime > 4
    }

    override fun done() {
        intake.intakeState = Intake.IntakeState.SLOW
    }

    override fun start() {
        startTime = Timer.getFPGATimestamp()
    }
}