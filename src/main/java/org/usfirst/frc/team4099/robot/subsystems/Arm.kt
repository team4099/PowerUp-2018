package org.usfirst.frc.team4099.robot.subsystems
import org.usfirst.frc.team4099.robot.Constants

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.loops.Loop
import edu.wpi.first.wpilibj.Timer
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
    private var armAngle = 0.0
    private var armBaseAngle = 0.0

    private var brakeTime = 0.0
    private var holdTime = 0.0


    var armState = ArmState.EXCHANGE


    var useVelocityControl : Boolean = false

    enum class MovementState {
        UP, STATIONARY, DOWN,  HOLD
    }

    enum class ArmState(val targetPos: Double) {
        LOW(-7* Math.PI / 6), EXCHANGE(0.0), HIGH(7 * Math.PI / 6), VELOCITY_CONTROL(Double.NaN), STILL(Double.NaN)
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
        armBaseAngle = ArmConversion.pulsesToRadians(masterSRX.sensorCollection.pulseWidthPosition)
        brakeTime = Timer.getFPGATimestamp()

    }


    /**
     * Changes power to the arm
     *
     * @param power A double for the power going to the arm
     */
    fun setArmVelocity(power: Double) {
        armState = ArmState.VELOCITY_CONTROL
        masterSRX.set(ControlMode.Velocity, Math.abs(power))
        armPower = power
    }

    private fun setArmPosition(position: Double) {
        armAngle = position
        masterSRX.set(ControlMode.MotionMagic, ArmConversion.radiansToPulses(position).toDouble())
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
        setArmVelocity(0.0)
        brake.set(DoubleSolenoid.Value.kReverse)
    }

    /**
     * Handles power to arm during each loop cycle
     *
     * @constructor Creates loop that controls power to arm during each loop cycle
     */
    val loop: Loop = object : Loop {

        override fun onStart() {
            movementState = MovementState.STATIONARY
            setArmVelocity(0.0)
            brake.set(DoubleSolenoid.Value.kForward)
        }

        /**
         * Handles events during each loop cycle, depends on arm control mode
         */
        override fun onLoop() {
            synchronized(this@Arm) {
                // if quadrature velocity is zero, set to hold
                // after 0.5 seconds of hold, actuate the brake and prevent movement
                // 0.5 seconds after actuating the brake, set to stationary
                // once the user tries to move, un-actuate the brake
                if (masterSRX.sensorCollection.quadratureVelocity == 0) {
                    movementState = Arm.MovementState.HOLD
                    armState = Arm.ArmState.STILL
                }
                if (movementState == Arm.MovementState.HOLD && Timer.getFPGATimestamp() - holdTime >= 0.5) {
                    brake.set(DoubleSolenoid.Value.kReverse)

                }
                if (movementState != Arm.MovementState.STATIONARY && brake.get() == DoubleSolenoid.Value.kReverse) {
                    brakeTime = Timer.getFPGATimestamp()
                    brake.set(DoubleSolenoid.Value.kForward)
                }
                if (Timer.getFPGATimestamp() - brakeTime < 0.5) {
                    return
                }
                //if current time - init time > 0.5 : skip loop: set to stationary
                if (movementState == Arm.MovementState.STATIONARY) {
                    brake.set(DoubleSolenoid.Value.kReverse)
                } else if (movementState == Arm.MovementState.HOLD) {
                    masterSRX.set(ControlMode.MotionMagic, ArmConversion.pulsesToRadians(masterSRX.sensorCollection.pulseWidthPosition))
                } else {
                    brake.set(DoubleSolenoid.Value.kForward)
                    when (armState) {
                        Arm.ArmState.LOW -> {
                            setArmPosition(ArmState.LOW.targetPos)
                        }
                        Arm.ArmState.EXCHANGE -> {
                            setArmPosition(ArmState.EXCHANGE.targetPos)
                        }
                        Arm.ArmState.HIGH -> {
                            setArmPosition(ArmState.HIGH.targetPos)
                        }
                            //masterSRX.sensorCollection.quadratureVelocity
                        Arm.ArmState.VELOCITY_CONTROL -> { }
                        Arm.ArmState.STILL -> {
                            masterSRX.set(ControlMode.MotionMagic, ArmConversion.pulsesToRadians(masterSRX.sensorCollection.pulseWidthPosition))
                        }
                    }
                }

                if (masterSRX.sensorCollection.quadratureVelocity > 0) {
                    movementState = Arm.MovementState.UP
                } else if (masterSRX.sensorCollection.quadratureVelocity == 0) {
                    //movementState = Arm.MovementState.HOLD
                    if (movementState != Arm.MovementState.STATIONARY && brake.get() == DoubleSolenoid.Value.kReverse) {
                        if (armState != Arm.ArmState.STILL) {
                            brake.set(DoubleSolenoid.Value.kForward)
                        }
                    } else {
                        movementState = Arm.MovementState.HOLD
                        holdTime = Timer.getFPGATimestamp()
                    }
                    if (Timer.getFPGATimestamp() - holdTime == 0.5) {
                        movementState = Arm.MovementState.STATIONARY
                    } else {
                        movementState = Arm.MovementState.DOWN
                    }
                    armAngle = ArmConversion.pulsesToRadians(masterSRX.sensorCollection.pulseWidthPosition) - armBaseAngle
                    if (movementState == Arm.MovementState.STATIONARY) {
                        brake.set(DoubleSolenoid.Value.kReverse)
                        masterSRX.set(ControlMode.Velocity, 0.0)

                    }
                }
            }
        }

        override fun onStop() = stop()

    }

    companion object {
        val instance = Arm()
    }

    override fun zeroSensors() {
        armBaseAngle = ArmConversion.pulsesToRadians(masterSRX.sensorCollection.pulseWidthPosition)
        armAngle = 0.0
    }

}