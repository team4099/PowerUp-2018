package org.usfirst.frc.team4099.robot.subsystems
import org.usfirst.frc.team4099.robot.Constants

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.loops.Loop
import edu.wpi.first.wpilibj.DoubleSolenoid
import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import kotlin.math.PI

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
    private var armAngle = 0.0

    var armState = ArmState.EXCHANGE


    var useVelocityControl : Boolean = false

    enum class MovementState {
        UP, STATIONARY, DOWN, MOVING_TO_EXCHANGE, MOVING_TO_TOP, MOVING_TO_BOTTOM
    }

    enum class ArmState(val targetPos: Double) {
        LOW(-7* PI / 6), EXCHANGE(0.0), HIGH(7 * PI / 6), VELOCITY_CONTROL(Double.NaN)
    }




    init {
        masterSRX.set(ControlMode.MotionMagic, 0.0)
        slaveSRX1.set(ControlMode.Follower, Constants.Arm.MASTER_SRX_ID.toDouble())
        slaveSRX2.set(ControlMode.Follower, Constants.Arm.MASTER_SRX_ID.toDouble())
        slaveSRX3.set(ControlMode.Follower, Constants.Arm.MASTER_SRX_ID.toDouble())
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


    /**
     * Changes power to the arm
     *
     * @param power A double for the power going to the arm
     */
    private fun setArmPower(power: Double) {
        armState = ArmState.VELOCITY_CONTROL
        masterSRX.set(ControlMode.MotionMagic, Math.abs(power))
        armPower = power
    }

    /**
     * Outputs arm subsystem
     *
     */
    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("armPower", armPower)
    }

    @Synchronized override fun stop() {
        movementState = MovementState.STATIONARY
        setArmPower(0.0)
    }

    /**
     * Handles power to arm during each loop cycle
     *
     * @constructor Creates loop that controls power to arm during each loop cycle
     */
    val loop: Loop = object : Loop {

        override fun onStart() {
            movementState = MovementState.STATIONARY
            setArmPower(0.0)
        }

        /**
         * Handles events during each loop cycle, depends on arm control mode
         */
        override fun onLoop() {
            synchronized(this@Arm) {
                when (armState) {
                    Arm.ArmState.LOW -> {
                        masterSRX.set(ControlMode.MotionMagic, ArmConversion.radiansToPulses(ArmState.LOW.targetPos).toDouble())
                        armAngle = ArmState.LOW.targetPos
                    }
                    Arm.ArmState.EXCHANGE -> {
                        masterSRX.set(ControlMode.MotionMagic, ArmConversion.radiansToPulses(ArmState.EXCHANGE.targetPos).toDouble())
                        armAngle = ArmState.EXCHANGE.targetPos
                    }
                    Arm.ArmState.HIGH -> {
                        masterSRX.set(ControlMode.MotionMagic, ArmConversion.radiansToPulses(ArmState.HIGH.targetPos).toDouble())
                        armAngle = ArmState.HIGH.targetPos
                    }
                    ArmState.VELOCITY_CONTROL -> {
                        when (movementState) {
                            Arm.MovementState.UP -> {
                                setArmPower(1.0)
                            }
                            Arm.MovementState.STATIONARY -> {
                                setArmPower(0.0)
                            }
                            Arm.MovementState.MOVING_TO_BOTTOM -> {
                                setArmPower(-1.0)
                            }
                            Arm.MovementState.MOVING_TO_EXCHANGE -> {
                                if (armAngle == ArmState.LOW.targetPos) {
                                    setArmPower(1.0)
                                } else if (armAngle == ArmState.HIGH.targetPos) {
                                    setArmPower(-1.0)
                                }
                            }
                            Arm.MovementState.MOVING_TO_TOP -> {
                                setArmPower(1.0)
                            }
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