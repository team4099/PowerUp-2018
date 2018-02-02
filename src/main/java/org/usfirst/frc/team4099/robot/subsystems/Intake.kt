package org.usfirst.frc.team4099.robot.subsystems

import edu.wpi.first.wpilibj.Talon
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Intake private constructor() : Subsystem {

    private val rightTalon = Talon(Constants.Intake.RIGHT_INTAKE_TALON_ID)
    private val leftTalon = Talon(Constants.Intake.LEFT_INTAKE_TALON_ID)

    var intakeState = IntakeState.STOP
    private var intakePower = 0.0

    enum class IntakeState {
        IN, STOP, OUT
    }

    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("intakePower", intakePower)
    }

    @Synchronized override fun stop() {
        intakeState = IntakeState.STOP
        setIntakePower(0.0)
    }

    private fun setIntakePower(power: Double) {
        rightTalon.set(Math.abs(power))
        leftTalon.set(Math.abs(power))
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            setIntakePower(0.0)
        }

        override fun onLoop() {
            synchronized(this@Intake) {
                when (intakeState) {
                    Intake.IntakeState.IN -> setIntakePower(-0.7)
                    Intake.IntakeState.STOP -> setIntakePower(0.0)
                    Intake.IntakeState.OUT -> setIntakePower(0.7)

                }
            }
        }

        override fun onStop() = stop()

    }

    companion object {
        val instance = Intake()
    }

    override fun zeroSensors() { }
}