package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
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
    private val talon = TalonSRX(Constants.Wrist.WRIST_TALON_ID)
//    private val arm = Arm.instance

    var wristState = WristState.HORIZONTAL
    private var wristPower = 0.0
    private var wristAngle = 0.0
//    private var outOfBounds: Boolean = true
//        get() = talon.motorOutputPercent > 0 && talon.sensorCollection.quadraturePosition < 0 ||
//                talon.motorOutputPercent < 0 && talon.sensorCollection.quadraturePosition > 1600

    enum class WristState(val targetAngle: Double) {
        HORIZONTAL(0.0),
        STOWED_UP(Math.PI / 2),
        SHOOT_UP(Math.PI / 4),
        OPEN_LOOP(Double.NaN)
        //TODO Calibrate values
    }

    init {
        talon.set(ControlMode.PercentOutput, 0.0)
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        talon.configNominalOutputForward(0.0, 0)
        talon.configNominalOutputReverse(0.0, 0)
        talon.configPeakOutputReverse(-1.0, 0)
        talon.configPeakOutputForward(1.0, 0)
        talon.config_kP(0, Constants.Gains.WRIST_KP, 0)
        talon.config_kI(0, Constants.Gains.WRIST_KI, 0)
        talon.config_kD(0, Constants.Gains.WRIST_KD, 0)
        talon.config_kF(0, Constants.Gains.WRIST_KF, 0)
        talon.configMotionCruiseVelocity(0, 0)
        talon.configMotionAcceleration(0, 0)
//        talon.configForwardSoftLimitEnable(true, 0)
//        talon.configForwardSoftLimitThreshold(0, 0)
//        talon.configReverseSoftLimitEnable(true, 0)
//        talon.configReverseSoftLimitThreshold(2048, 0)
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
    fun setWristMode(state: WristState) {
        wristState = state
    }

    fun getWristPosition() : Double {
        return WristConversion.pulsesToRadians(talon.sensorCollection.quadraturePosition)
    }

    fun setOpenLoop(power: Double) {
//        if (outOfBounds) {
//            return
//        }
        wristState = WristState.OPEN_LOOP
        wristPower = power
        talon.set(ControlMode.PercentOutput, power)
    }

    val loop: Loop = object : Loop {
        override fun onStart() {}

        override fun onLoop() {
            synchronized(this@Wrist) {
//                if (outOfBounds) {
//                    wristPower = 0.0
//                    talon.set(ControlMode.PercentOutput, 0.0)
//                    return
//                }
                if (wristState == WristState.OPEN_LOOP) {
                    return
                }
                talon.set(ControlMode.MotionMagic, wristState.targetAngle / WristConversion.pulsesToRadians)
            }
        }

        override fun onStop() = stop()

    }

    override fun zeroSensors() {
        talon.sensorCollection.setQuadraturePosition(0, 0)
    }

    companion object {
        val instance = Wrist()
    }

}
