package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import edu.wpi.first.wpilibj.Talon
import org.usfirst.frc.team4099.lib.util.CANMotorControllerFactory
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop


class Climber private constructor(): Subsystem {
    enum class ClimberState {
        CLIMBING, NOT_CLIMBING, UNCLIMBING
    }

    var climberState = ClimberState.NOT_CLIMBING

    val talon = Talon(Constants.Climber.CLIMBER_TALON_ID)

    override fun outputToSmartDashboard() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun stop() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun zeroSensors() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
        }

        override fun onLoop() {
            synchronized(this@Climber) {
                when(climberState) {
                    ClimberState.CLIMBING -> talon.set(-1.0)
                    ClimberState.NOT_CLIMBING -> talon.set(0.0)
                    ClimberState.UNCLIMBING -> talon.set(1.0)
                }
            }
        }

        override fun onStop() {
        }
    }

    companion object {
        val instance = Climber()
    }

}