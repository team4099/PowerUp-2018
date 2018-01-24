package org.usfirst.frc.team4099.robot.subsystems
import com.ctre.phoenix.motorcontrol.ControlMode
import org.usfirst.frc.team4099.robot.Constants

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.loops.Loop

class Wrist private constructor(): Subsystem {
    private val wristSRX = TalonSRX(Constants.Wrist.WRIST_TALON_ID)
    private val arm = Arm.instance

    var wristState = WristState.HORIZONTAL
    private var wristPower = 0.0
    private var wristAngle = 0.0


    enum class WristState(wristAngle: Double) {
        STOWED_DOWN(-7*Math.PI/6),
        HORIZONTAL(Double.NaN),
        STOWED_UP(7*Math.PI)
    }

    init {
        wristSRX.set(ControlMode.MotionMagic, 0.0)
    }


    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("wristPower", wristPower)
    }

    @Synchronized override fun stop() {
        setWristMode(WristState.HORIZONTAL)
        setWristPower(0.0)
    }

    private fun setWristMode(state: WristState) {
        wristState = state
    }

    private fun setWristPower(power: Double) {
        wristSRX.set(ControlMode.MotionMagic, power)
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            setWristPower(0.0)
        }

        override fun onLoop() {
            if (wristState == WristState.HORIZONTAL) {
                wristAngle = 2*Math.PI - arm.armAngle
            }
            synchronized(this@Wrist) {
                wristSRX.set(ControlMode.MotionMagic, WristConversion.radiansToPulses(wristAngle).toDouble())
            }
        }

        override fun onStop() = stop()

    }

    override fun zeroSensors() {    }

}
