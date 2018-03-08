package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.util.CANMotorControllerFactory
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Elevator private constructor(): Subsystem {
    val talon = CANMotorControllerFactory.createDefaultTalon(Constants.Elevator.ELEVATOR_TALON_ID)

    private var elevatorPower = 0.0
    var elevatorState = ElevatorState.OPEN_LOOP
    var movementState = MovementState.STILL
        private set
    var observedElevatorPosition = 0.0
        private set
    var observedElevatorVelocity = 0.0
        private set

    enum class ElevatorState (val targetPos : Double) {
        LOW(2.0), MEDIUM(0.5), HIGH(15.0), VELOCITY_CONTROL(Double.NaN), OPEN_LOOP(Double.NaN)
    }

    enum class MovementState {
        UP, DOWN, STILL
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
//        println("power: $power")
        elevatorState = ElevatorState.OPEN_LOOP
        talon.set(ControlMode.PercentOutput, power)
//        println("output ${talon.outputCurrent}")
//        println("elevator position: ${talon.sensorCollection.quadraturePosition}")
//        println("elevator inches: ${ElevatorConversion.pulsesToInches(talon.sensorCollection.quadraturePosition)}")
//        println("elevator speed: ${talon.sensorCollection. quadratureVelocity}")
    }


    fun setElevatorVelocity(inchesPerSecond: Double) {
        elevatorState = ElevatorState.VELOCITY_CONTROL
        talon.set(ControlMode.Velocity, inchesPerSecond)
    }


    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("elevatorPower", elevatorPower)
    }

    private fun setElevatorPosition(position: ElevatorState) {
        var target = position.targetPos
        if (target == Double.NaN) {
            target = observedElevatorPosition
        }
        observedElevatorPosition = target
        talon.set(ControlMode.MotionMagic, ElevatorConversion.inchesToPulses(target).toDouble())
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            elevatorState = ElevatorState.LOW
        }

        override fun onLoop() {
            synchronized(this@Elevator) {
                observedElevatorVelocity = ElevatorConversion.nativeSpeedToInchesPerSecond(talon.sensorCollection.quadratureVelocity)
                observedElevatorPosition = ElevatorConversion.pulsesToInches(talon.sensorCollection.quadraturePosition)
                elevatorPower = talon.motorOutputPercent

                when (elevatorState){
                    ElevatorState.OPEN_LOOP -> {
                        return
                    }
                    ElevatorState.VELOCITY_CONTROL -> {
                        return
                    }
                    ElevatorState.HIGH -> {
                        setElevatorPosition(ElevatorState.HIGH)
                    }
                    ElevatorState.LOW -> {
                        setElevatorPosition(ElevatorState.LOW)
                    }
                    ElevatorState.MEDIUM -> {
                        setElevatorPosition(ElevatorState.MEDIUM)
                    }
                }
                when {
                    observedElevatorVelocity in -1 .. 1 -> movementState = MovementState.STILL
                    observedElevatorVelocity > 1 -> movementState = MovementState.UP
                    observedElevatorVelocity < 1 -> movementState = MovementState.DOWN
                }
            }
        }

        override fun onStop() {
            setElevatorVelocity(0.0)
        }
    }

    override fun stop() {
        movementState = Elevator.MovementState.STILL
        setElevatorVelocity(0.0)

    }

    companion object {
        val instance = Elevator()
    }

    override fun zeroSensors() {
        talon.sensorCollection.setQuadraturePosition(0, 0)
    }
}