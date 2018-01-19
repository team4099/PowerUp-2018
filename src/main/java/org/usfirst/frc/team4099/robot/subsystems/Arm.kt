package org.usfirst.frc.team4099.robot.subsystems
import org.usfirst.frc.team4099.robot.Constants

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.loops.Loop
import edu.wpi.first.wpilibj.DoubleSolenoid
import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX

/**
 * @author Team 4099
 *
 * This class is the constructor for the Arm subsystem
 *
 * @constructor Creates the Arm subsystem
 *
 */
class Arm private constructor() : Subsystem {

    private val masterSRX = TalonSRX(Constants.Arm.MASTER_SRX_ID)
    private val slaveSRX1 = TalonSRX(Constants.Arm.SLAVE_SRX_1_ID)
    private val slaveSRX2 = TalonSRX(Constants.Arm.SLAVE_SRX_2_ID)
    private val slaveSRX3 = TalonSRX(Constants.Arm.SLAVE_SRX_3_ID)

    private val brake = DoubleSolenoid(Constants.Arm.BRAKE_FORWARD_CHANNEL, Constants.Arm.BRAKE_REVERSE_CHANNEL)


    var movementState = MovementState.STATIONARY
    private var armPower = 0.0

    var armState = ArmState.EXCHANGE
    var targetPos: Double = 0.0

    var useVelocityControl : Boolean = false

    enum class MovementState {
        UP, STATIONARY, DOWN
    }

    enum class ArmState {
        LOW, EXCHANGE, HIGH
    }




    init {
        masterSRX.set(ControlMode.MotionMagic, Constants.Arm.MASTER_SRX_ID.toDouble())
        slaveSRX1.set(ControlMode.Follower, Constants.Arm.MASTER_SRX_ID.toDouble())
        slaveSRX2.set(ControlMode.Follower, Constants.Arm.MASTER_SRX_ID.toDouble())
        slaveSRX3.set(ControlMode.Follower, Constants.Arm.MASTER_SRX_ID.toDouble())
        if (useVelocityControl) {
            masterSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)
            masterSRX.setSensorPhase(false)
            masterSRX.configNominalOutputForward(+0.0, 0)
            masterSRX.configPeakOutputForward(+1.0, 0)
            masterSRX.selectProfileSlot(0, 0)
            masterSRX.config_kF(0, 0.0, 0)
            masterSRX.config_kP(0, 0.0, 0)
            masterSRX.config_kI(0, 0.0, 0)
            masterSRX.config_kD(0, 0.0, 0)
        }
    }

    fun velocityControlSwitch(velocityControl: Boolean) {
        useVelocityControl = velocityControl
    }

    /**
     * Changes power to the arm
     * @param[power] double power
     */
    private fun setArmPower(power: Double) {
        masterSRX.set(ControlMode.MotionMagic, Math.abs(power))
    }

    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("armPower", armPower)
    }

    @Synchronized override fun stop() {
        movementState = MovementState.STATIONARY
        setArmPower(0.0)
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            setArmPower(0.0)
        }

        override fun onLoop() {
            synchronized(this@Arm) {
                if ( !useVelocityControl) {
                    when (movementState) {
                        Arm.MovementState.DOWN -> {
                            setArmPower(-1.0)
                            brake.set(DoubleSolenoid.Value.kForward)
                        }
                        Arm.MovementState.STATIONARY -> {var targetPos: Double
                            setArmPower(0.0)
                            brake.set(DoubleSolenoid.Value.kReverse)
                        }
                        Arm.MovementState.UP -> {
                            setArmPower(1.0)
                            brake.set(DoubleSolenoid.Value.kForward)
                        }
                    }
                } else {
                    when (armState) {
                        Arm.ArmState.LOW -> {
                            targetPos = -70*Math.PI/360
                            masterSRX.set(ControlMode.MotionMagic, ArmConversion.radiansToPulses(targetPos).toDouble())
                        }
                        Arm.ArmState.EXCHANGE -> {
                            targetPos = Math.PI/6
                            masterSRX.set(ControlMode.MotionMagic, ArmConversion.radiansToPulses(targetPos).toDouble())
                        }
                        Arm.ArmState.HIGH -> {
                            targetPos = 70*Math.PI/360
                            masterSRX.set(ControlMode.MotionMagic, ArmConversion.radiansToPulses(targetPos).toDouble())
                        }
                    }


                }
            }
        }

        override fun onStop() = stop()

    }

    companion object {
        val instance = Arm()
    }

    override fun zeroSensors() { }

}