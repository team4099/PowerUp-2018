package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.util.CrashTracker
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Climber private constructor() : Subsystem {
    private val climberTalon = TalonSRX(Constants.Climber.CLIMBER_TALON_ID)
    private val climberPower = 0.0

    private val latch = DoubleSolenoid(Constants.Climber.FORWARD_CHANNEL,Constants.Climber.BACKWARD_CHANNEL)
    private var noLatch = false



    var climberState = ClimberState.NOT_CLIMBING
        private set

    enum class ClimberState {
        CLIMBING, NOT_CLIMBING
    }

    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("climberPower", climberPower)
    }

    @Synchronized override fun stop() {
        setClimbingMode(ClimberState.NOT_CLIMBING)
        setClimberPower(0.0)
    }

    override fun zeroSensors() {}

    fun setClimbingMode(state: ClimberState) {
        climberState = state
    }

    private fun setClimberPower(power: Double) {
        climberTalon.set(ControlMode.Velocity, -Math.abs(power))
    }

    fun getLatch() : Boolean {
        return noLatch
    }

    fun setLatch(wantsLatch: Boolean) {
        if (wantsLatch != noLatch) {
            noLatch = wantsLatch
        }
        if (wantsLatch) {
            latch.set(DoubleSolenoid.Value.kForward)
        } else {
            latch.set(DoubleSolenoid.Value.kReverse)
        }
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            setClimberPower(0.0)
        }

        override fun onLoop() {
            synchronized(this@Climber) {
                when (climberState) {
                    Climber.ClimberState.CLIMBING -> setClimberPower(1.0)
                    Climber.ClimberState.NOT_CLIMBING -> setClimberPower(0.0)
                    else -> {
                        CrashTracker.logMarker("REACHED ILLEGAL STATE IN CLIMBER")
                        setClimberPower(0.0)
                        climberState = ClimberState.NOT_CLIMBING
                    }
                }
            }
        }

        override fun onStop() {
            stop()
        }
    }

    companion object {
        val instance = Climber()
    }

}