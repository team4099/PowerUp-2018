package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop
import java.text.FieldPosition

class Elevator private constructor(): Subsystem{
    private val masterSRX = TalonSRX(Constants.Elevator.ELEVATOR_MASTER_TALON_ID)
    private val slaveSRX = TalonSRX(Constants.Elevator.ELEVATOR_SLAVE_TALON_ID)

    private var elevatorPower = 0.0
    var elevatorState = ElevatorState.LOW
    private var movementState = MovementState.HOLD
    private var elevatorPosition = 0.0
    private var limitSwitch = DigitalInput(0)

    enum class ElevatorState (val targetPos : Double) {
        LOW(0.0), MEDIUM(0.5), HIGH(1.0), STILL(Double.NaN), VELOCITY_CONTROL(Double.NaN)
    }

    enum class MovementState {
        UP, DOWN, HOLD
    }

    init {
        masterSRX.set(ControlMode.Velocity, 0.0)
        slaveSRX.set(ControlMode.Follower, 0.0)
    }

    fun setElevatorVelocity(power: Double) {
        elevatorState = ElevatorState.VELOCITY_CONTROL
        masterSRX.set(ControlMode.Velocity, Math.abs(power))
        elevatorPower = power

    }


    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("elevatorPower", elevatorPower)
    }

    private fun setElevatorPosition(position : Double) {
        elevatorPosition = position
        masterSRX.set(ControlMode.MotionMagic, ElevatorConversion.radiansToPulses(position).toDouble())
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            setElevatorVelocity(0.0)
            movementState = Elevator.MovementState.HOLD
        }

        override fun onLoop() {
            synchronized(this@Elevator) {
                if (movementState != Elevator.MovementState.HOLD && masterSRX.sensorCollection.quadratureVelocity == 0) {
                    movementState = Elevator.MovementState.HOLD
                }
                elevatorPosition = ElevatorConversion.pulsesToRadians(masterSRX.sensorCollection.quadraturePosition)

                if (movementState == Elevator.MovementState.HOLD){
                    masterSRX.set(ControlMode.MotionMagic, elevatorPosition)
                } else {
                    when(elevatorState){
                        ElevatorState.VELOCITY_CONTROL -> {
                            masterSRX.set(ControlMode.Velocity, 0.0)
                            if(limitSwitch.get() == true) {
                                when (movementState) {
                                    MovementState.UP -> setElevatorVelocity(1.0)
                                    MovementState.DOWN -> setElevatorVelocity(-1.0)
                                    MovementState.HOLD -> {
                                        masterSRX.set(ControlMode.MotionMagic, ElevatorConversion.pulsesToRadians(masterSRX.sensorCollection.pulseWidthPosition))
                                    }
                                }
                            } else {
                                movementState = MovementState.HOLD
                                masterSRX.set(ControlMode.MotionMagic, ElevatorConversion.pulsesToRadians(masterSRX.sensorCollection.pulseWidthPosition))
                            }
                        }
                        ElevatorState.HIGH -> {
                            setElevatorPosition(Elevator.ElevatorState.HIGH.targetPos)
                        }
                        ElevatorState.LOW -> {
                            setElevatorPosition(Elevator.ElevatorState.LOW.targetPos)
                        }
                        ElevatorState.MEDIUM -> {
                            setElevatorPosition(Elevator.ElevatorState.MEDIUM.targetPos)
                        }
                        ElevatorState.STILL -> {
                            masterSRX.set(ControlMode.MotionMagic, ElevatorConversion.radiansToPulses(ElevatorState.STILL.targetPos).toDouble())
                        }
                    }
                }
            }

        }

        override fun onStop() {
            movementState = MovementState.HOLD
            setElevatorPosition(ElevatorState.STILL.targetPos)
        }
    }

    override fun stop() {
        movementState = Elevator.MovementState.HOLD
        masterSRX.set(ControlMode.Velocity, 0.0)

    }

    companion object {
        val instance = Elevator()
    }

    override fun zeroSensors() {
        TODO("not implemented") //not needed?
    }
}