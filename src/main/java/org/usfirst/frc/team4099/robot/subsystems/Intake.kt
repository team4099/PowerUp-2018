package org.usfirst.frc.team4099.robot.subsystems
import com.ctre.phoenix.motorcontrol.ControlMode
import org.usfirst.frc.team4099.robot.Constants

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.loops.Loop

class Intake private constructor() : Subsystem {

    private val rightTalon = TalonSRX(Constants.Intake.RIGHT_INTAKE_TALON_ID)
    private val leftTalon = TalonSRX(Constants.Intake.LEFT_INTAKE_TALON_ID)

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
        rightTalon.set(ControlMode.Velocity, Math.abs(power))
        leftTalon.set(ControlMode.Velocity,-Math.abs(power))
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            setIntakePower(0.0)
        }

        override fun onLoop() {
            synchronized(this@Intake) {
                when (intakeState) {
                    Intake.IntakeState.IN -> setIntakePower(-1.0)
                    Intake.IntakeState.STOP -> setIntakePower(0.0)
                    Intake.IntakeState.OUT -> setIntakePower(1.0)

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