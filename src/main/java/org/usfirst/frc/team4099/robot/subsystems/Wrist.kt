package org.usfirst.frc.team4099.robot.subsystems
import com.ctre.phoenix.motorcontrol.ControlMode
import org.usfirst.frc.team4099.robot.Constants

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.loops.Loop

/**
 * @author Team 4099
 *
 * This class is the constructor for the Wrist subsystem
 *
 * @constructor Creates the Wrist subsystem
 *
 */

class Wrist private constructor(): Subsystem {
    private val wristSRX = TalonSRX(Constants.Wrist.WRIST_TALON_ID)
    private val arm = Arm.instance

    var wristState = WristState.HORIZONTAL
    private var wristPower = 0.0
    private var wristAngle = 0.0


    enum class WristState(wristAngle: Double) {
        STOWED_DOWN(-7*Math.PI/6),
        HORIZONTAL(Double.NaN),
        STOWED_UP(7*Math.PI / 6),
        SHOOT_UP(Math.PI/3),
        CLIMBING(Math.PI/2)
    }

    init {
        wristSRX.set(ControlMode.MotionMagic, 0.0)
    }

    /**
     * Outputs the angle of the wrist
     */
    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("wristAngle", wristAngle)
    }

    @Synchronized override fun stop() {
        setWristMode(WristState.HORIZONTAL)
    }

    /**
     * Sets the state of the Arm
     *
     * @param state is the wrist state
     */
    private fun setWristMode(state: WristState) {
        wristState = state
    }


    val loop: Loop = object : Loop {
        override fun onStart() {}

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
