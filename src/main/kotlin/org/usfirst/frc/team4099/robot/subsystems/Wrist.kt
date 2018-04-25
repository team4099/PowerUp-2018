package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.util.CANMotorControllerFactory
import org.usfirst.frc.team4099.lib.util.conversions.WristConversion
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop
import java.lang.Math.abs

/**
 * @author Team 4099
 *
 * This class is the constructor for the Wrist subsystem
 *
 * @constructor Creates the Wrist subsystem
 *
 */

class Wrist private constructor(): Subsystem {
    private val talon = CANMotorControllerFactory.createDefaultTalon(Constants.Wrist.WRIST_TALON_ID)
//    private val arm = Arm.instance

    var wristState = WristState.HORIZONTAL
    private var wristPower = 0.0
    private var wristAngle = 0.0
//    private var outOfBounds: Boolean = true
//        get() = talon.motorOutputPercent > 0 && talon.sensorCollection.quadraturePosition < 0 ||
//                talon.motorOutputPercent < 0 && talon.sensorCollection.quadraturePosition > 1600

    private var tooHigh = false
    private var tooLow = false

    enum class WristState(val targetAngle: Double) {
        HORIZONTAL(0.0),
        STOWED_UP(Math.PI / 2),
        SHOOT_UP(Math.PI / 4),
        OPEN_LOOP(Double.NaN),
        VELOCITY_CONTROL(Double.NaN)
        //TODO Calibrate values
    }

    init {
        talon.set(ControlMode.PercentOutput, 0.0)
        talon.inverted = true
        talon.setSensorPhase(true)
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        talon.configNominalOutputForward(0.0, 0)
        talon.configNominalOutputReverse(0.0, 0)
        talon.configPeakOutputReverse(-1.0, 0)
        talon.configPeakOutputForward(1.0, 0)

        talon.config_kP(0, Constants.Wrist.WRIST_UP_KP, 0)
        talon.config_kI(0, Constants.Wrist.WRIST_UP_KI, 0)
        talon.config_kD(0, Constants.Wrist.WRIST_UP_KD, 0)
        talon.config_kF(0, Constants.Wrist.WRIST_UP_KF, 0)

        talon.config_kP(1, Constants.Wrist.WRIST_DOWN_KP, 0)
        talon.config_kI(1, Constants.Wrist.WRIST_DOWN_KI, 0)
        talon.config_kD(1, Constants.Wrist.WRIST_DOWN_KD, 0)
        talon.config_kF(1, Constants.Wrist.WRIST_DOWN_KF, 0)

        talon.configMotionCruiseVelocity(0, 0)
        talon.configMotionAcceleration(0, 0)
        talon.configForwardSoftLimitEnable(true, 0)
        talon.configForwardSoftLimitThreshold(100, 0)
        talon.configReverseSoftLimitEnable(true, 0)
        talon.configReverseSoftLimitThreshold(0, 0)
        talon.overrideSoftLimitsEnable(false)
        talon.overrideLimitSwitchesEnable(true)
    }

    /**
     * Outputs the angle of the wrist
     */
    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("wrist/wristAngle", wristAngle)
        SmartDashboard.putBoolean("wrist/wristUp", wristAngle > Math.PI / 4)
        SmartDashboard.putNumber("wrist/wristSpeed", talon.sensorCollection.quadratureVelocity.toDouble())
    }

    @Synchronized override fun stop() {
//        setWristMode(WristState.HORIZONTAL)
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
        return WristConversion.pulsesToRadians(talon.sensorCollection.quadraturePosition.toDouble())
    }

    fun setOpenLoop(power: Double) {
        wristState = WristState.OPEN_LOOP
        wristPower = power
        talon.set(ControlMode.PercentOutput, wristPower)
        println("wrist speed: ${talon.sensorCollection.quadratureVelocity}")
    }

    fun setWristVelocity(radiansPerSecond: Double) {
//        if ((radiansPerSecond <= 0 || Utils.around(radiansPerSecond, 0.0, .1)) && talon.sensorCollection.quadraturePosition < 2.5) {
//            setOpenLoop(0.0)
//            println("wrist exiting at 0 power, $radiansPerSecond")
//            return
//        }
        wristState = WristState.VELOCITY_CONTROL
        if(radiansPerSecond > 0) {
            talon.selectProfileSlot(1, 0)
        } else {
            talon.selectProfileSlot(0, 0)
        }
        talon.set(ControlMode.Velocity, radiansPerSecond)
        println("nativeVel: $radiansPerSecond, observedVel: ${talon.sensorCollection.quadratureVelocity}, error: ${talon.sensorCollection.quadratureVelocity - radiansPerSecond}")

    }


    val loop: Loop = object : Loop {
        override fun onStart() {
            zeroSensors()
        }

        override fun onLoop() {
            synchronized(this@Wrist) {
                wristAngle = WristConversion.pulsesToRadians(talon.sensorCollection.quadraturePosition.toDouble())
                if (wristState == WristState.OPEN_LOOP || wristState == WristState.VELOCITY_CONTROL) {
                    return
                }
//                if (outOfBounds()) {
//                    wristPower = 0.0
//                    talon.set(ControlMode.PercentOutput, 0.0)
//                    return
//                }
                talon.set(ControlMode.MotionMagic, WristConversion.radiansToPulses(wristState.targetAngle).toDouble())

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
