package org.usfirst.frc.team4099.robot.subsystems
import org.usfirst.frc.team4099.robot.Constants

import edu.wpi.first.wpilibj.Talon
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.loops.Loop

class Wrist private constructor(): Subsystem {
    private val wristSRX = Talon(Constants.Wrist.WRIST_TALON_ID)

    var wristState = WristState.HORIZONTAL
    private var wristPower = 0.0

    enum class WristState {
        STOWED_DOWN, HORIZONTAL, STOWED_UP
    }

    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("wristPower", wristPower)
    }

    @Synchronized override fun stop() {
        setWristMode(WristState.HORIZONTAL)
        setWristPower(0.0)
    }

    fun setWristMode(state: WristState) {
        wristState = state
    }

    fun setWristPower(power: Double) {
        wristSRX.set(power)
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            setWristPower(0.0)
        }

        override fun onLoop() {
            synchronized(this@Wrist) {
                when (wristState) {
                    Wrist.WristState.HORIZONTAL -> {
                        setWristPower(0.0)
                    }
                    Wrist.WristState.STOWED_UP -> {
                        setWristPower(1.0)
                    }
                    Wrist.WristState.STOWED_DOWN -> {
                        setWristPower(-1.0)
                    }
                }
            }
        }

        override fun onStop() = stop()

    }

    override fun zeroSensors() {    }

}
