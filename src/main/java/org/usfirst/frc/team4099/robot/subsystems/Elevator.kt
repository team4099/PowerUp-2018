package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Elevator private constructor(): Subsystem {
    val talon = TalonSRX(Constants.Elevator.ELEVATOR_TALON_ID)

    private var elevatorPower = 0.0
    var elevatorState = ElevatorState.OPEN_LOOP
    var movementState = MovementState.HOLD
    private var elevatorPosition = 0.0

    enum class ElevatorState (val targetPos : Double) {
        LOW(2.0), MEDIUM(0.5), HIGH(15.0), STILL(Double.NaN), VELOCITY_CONTROL(Double.NaN), OPEN_LOOP(Double.NaN)
    }

    enum class MovementState {
        UP, DOWN, HOLD
    }

    init {
        talon.inverted = true
        talon.setSensorPhase(true)
        talon.set(ControlMode.Velocity, 0.0)
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        talon.configNominalOutputForward(0.0, 0)
        talon.configNominalOutputReverse(0.0, 0)
        talon.configPeakOutputReverse(-1.0, 0)
        talon.configPeakOutputForward(1.0, 0)
        talon.config_kP(0, Constants.Gains.ELEVATOR_KP, 0)
        talon.config_kI(0, Constants.Gains.ELEVATOR_KI, 0)
        talon.config_kD(0, Constants.Gains.ELEVATOR_KD, 0)
        talon.config_kF(0, Constants.Gains.ELEVATOR_KF, 0)
        talon.configMotionCruiseVelocity(0, 0)
        talon.configMotionAcceleration(0, 0)
    }

    fun setOpenLoop(power: Double) {
        println("power: $power")
        elevatorState = ElevatorState.OPEN_LOOP
        talon.set(ControlMode.PercentOutput, power)
        println(talon.outputCurrent)
//        println("elevator position: ${talon.sensorCollection.quadraturePosition}")
//        println("elevator inches: ${ElevatorConversion.pulsesToInches(talon.sensorCollection.quadraturePosition)}")
//        println("elevator speed: ${talon.sensorCollection. quadratureVelocity}")
    }


    fun setElevatorVelocity(power: Double) {
        elevatorState = ElevatorState.VELOCITY_CONTROL
        talon.set(ControlMode.Velocity, power)
        elevatorPower = power

    }


    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("elevatorPower", elevatorPower)
    }

    private fun setElevatorPosition(position : Double) {
        elevatorPosition = position
        talon.set(ControlMode.MotionMagic, ElevatorConversion.inchesToPulses(position).toDouble())
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            setElevatorVelocity(0.0)
            movementState = Elevator.MovementState.HOLD
        }

        override fun onLoop() {
            synchronized(this@Elevator) {
                if (elevatorState == ElevatorState.OPEN_LOOP) {
                    return
                }

                if (movementState != Elevator.MovementState.HOLD && talon.sensorCollection.quadratureVelocity == 0) {
                    movementState = Elevator.MovementState.HOLD
                }
                elevatorPosition = ElevatorConversion.pulsesToInches(talon.sensorCollection.quadraturePosition)


                if (movementState == Elevator.MovementState.HOLD){
                    talon.set(ControlMode.MotionMagic, elevatorPosition)
                } else {
                    when(elevatorState){
                        ElevatorState.VELOCITY_CONTROL -> {
                            talon.set(ControlMode.Velocity, 0.0)
                            if (/** limitSwitch.get() == true **/ false) {
                                when (movementState) {
                                    MovementState.UP -> setElevatorVelocity(1.0)
                                    MovementState.DOWN -> setElevatorVelocity(-1.0)
                                    MovementState.HOLD -> {
                                        talon.set(ControlMode.MotionMagic, ElevatorConversion.pulsesToInches(talon.sensorCollection.pulseWidthPosition))
                                    }
                                }
                            } else {
                                movementState = MovementState.HOLD
                                talon.set(ControlMode.MotionMagic, ElevatorConversion.pulsesToInches(talon.sensorCollection.pulseWidthPosition))
                            }
                        }
                        ElevatorState.OPEN_LOOP -> {
                            return
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
                            talon.set(ControlMode.MotionMagic, ElevatorConversion.inchesToPulses(ElevatorState.STILL.targetPos).toDouble())
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
        talon.set(ControlMode.Velocity, 0.0)

    }

    companion object {
        val instance = Elevator()
    }

    override fun zeroSensors() {
        talon.sensorCollection.setQuadraturePosition(0, 0)
    }
}