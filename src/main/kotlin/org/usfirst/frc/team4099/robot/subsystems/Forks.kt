package org.usfirst.frc.team4099.robot.subsystems

import edu.wpi.first.wpilibj.DoubleSolenoid
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Forks private constructor(): Subsystem {
    private val pneumaticLatch = DoubleSolenoid(Constants.Forks.LATCH_FORWARD_ID, Constants.Forks.LATCH_REVERSE_ID)

    var latched = false
        set(state) {
            pneumaticLatch.set(if (state) DoubleSolenoid.Value.kForward else DoubleSolenoid.Value.kReverse)
            field = state
        }

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
            latched = true
        }

        override fun onLoop() {
            synchronized(this@Forks) {

            }
        }

        override fun onStop() {
        }
    }

    companion object {
        val instance = Forks()
    }

}