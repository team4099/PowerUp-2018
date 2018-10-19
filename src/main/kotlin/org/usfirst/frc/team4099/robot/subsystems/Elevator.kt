package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.util.CANMotorControllerFactory
import org.usfirst.frc.team4099.lib.util.Utils
import org.usfirst.frc.team4099.lib.util.conversions.ElevatorConversion
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Elevator private constructor(): Subsystem {
    private val talon = CANMotorControllerFactory.createDefaultTalon(Constants.Elevator.ELEVATOR_TALON_ID)

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
        talon.inverted = false
        talon.clearStickyFaults(0)
        talon.setSensorPhase(false)
        talon.set(ControlMode.PercentOutput, 0.0)
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        talon.configNominalOutputForward(0.0, 0)
        talon.configNominalOutputReverse(0.0, 0)
        talon.configPeakOutputReverse(-1.0, 0)
        talon.configPeakOutputForward(0.0, 0)
        talon.config_kP(0, Constants.Gains.ELEVATOR_UP_KP, 0)
        talon.config_kI(0, Constants.Gains.ELEVATOR_UP_KI, 0)
        talon.config_kD(0, Constants.Gains.ELEVATOR_UP_KD, 0)
        talon.config_kF(0, Constants.Gains.ELEVATOR_UP_KF, 0)

        talon.config_kP(1, Constants.Gains.ELEVATOR_DOWN_KP, 0)
        talon.config_kI(1, Constants.Gains.ELEVATOR_DOWN_KI, 0)
        talon.config_kD(1, Constants.Gains.ELEVATOR_DOWN_KD, 0)
        talon.config_kF(1, Constants.Gains.ELEVATOR_DOWN_KF, 0)


        talon.configMotionCruiseVelocity(0, 0)
        talon.configMotionAcceleration(0, 0)

        talon.configReverseSoftLimitEnable(true, 0)
        talon.configReverseSoftLimitThreshold(-ElevatorConversion.inchesToPulses(70.0).toInt(), 0)
        talon.overrideSoftLimitsEnable(true)

        SmartDashboard.putNumber("elevator/pidPDown", Constants.Gains.ELEVATOR_DOWN_KP)
        SmartDashboard.putNumber("elevator/pidIDown", Constants.Gains.ELEVATOR_DOWN_KI)
        SmartDashboard.putNumber("elevator/pidDDown", Constants.Gains.ELEVATOR_DOWN_KD)
        SmartDashboard.putNumber("elevator/pidFDown", Constants.Gains.ELEVATOR_DOWN_KF)

        SmartDashboard.putNumber("elevator/pidPUP", Constants.Gains.ELEVATOR_UP_KP)
        SmartDashboard.putNumber("elevator/pidIUP", Constants.Gains.ELEVATOR_UP_KI)
        SmartDashboard.putNumber("elevator/pidDUP", Constants.Gains.ELEVATOR_UP_KD)
        SmartDashboard.putNumber("elevator/pidFUP", Constants.Gains.ELEVATOR_UP_KF)
    }

    fun setOpenLoop(power: Double) {
//        println("power: $power")
        elevatorState = ElevatorState.OPEN_LOOP
        talon.set(ControlMode.PercentOutput, -power)
//        println("output ${talon.outputCurrent}")
//        println("elevator position: ${talon.sensorCollection.quadraturePosition}")
//        println("elevator inches: ${ElevatorConversion.pulsesToInches(talon.sensorCollection.quadraturePosition)}")
//        println("elevator speed: ${talon.sensorCollection. quadratureVelocity}")
    }


    fun setElevatorVelocity(inchesPerSecond: Double) {
        if ((inchesPerSecond <= 0 || Utils.around(inchesPerSecond, 0.0, .1)) && observedElevatorPosition < 0.5) {
            setOpenLoop(0.0)
            talon.sensorCollection.setQuadraturePosition(0, 0)
//            println("exiting at 0 power, $inchesPerSecond")
            return
        }
        elevatorState = ElevatorState.VELOCITY_CONTROL
        if(inchesPerSecond >= 0) {
            talon.selectProfileSlot(0, 0)
        } else {
            talon.selectProfileSlot(1, 0)
        }
        talon.set(ControlMode.Velocity, -inchesPerSecond)
//        println("nativeVel: $inchesPerSecond, observedVel: ${talon.sensorCollection.quadratureVelocity}")
    }


    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("elevator/elevatorVoltage", talon.motorOutputVoltage)
        SmartDashboard.putNumber("elevator/elevatorVelocity", talon.sensorCollection.quadratureVelocity.toDouble())
        SmartDashboard.putNumber("elevator/elevatorHeight", observedElevatorPosition)
        SmartDashboard.putNumber("elevator/closedLoopError", talon.getClosedLoopError(0).toDouble())


//        talon.config_kP(1, SmartDashboard.getNumber("elevator/pidPDown", Constants.Gains.ELEVATOR_DOWN_KP), 0)
//        talon.config_kI(1, SmartDashboard.getNumber("elevator/pidIDown", Constants.Gains.ELEVATOR_DOWN_KI), 0)
//        talon.config_kD(1, SmartDashboard.getNumber("elevator/pidDDown", Constants.Gains.ELEVATOR_DOWN_KD), 0)
//        talon.config_kF(1, SmartDashboard.getNumber("elevator/pidFDown", Constants.Gains.ELEVATOR_DOWN_KF), 0)
//
//
//        talon.config_kP(0, SmartDashboard.getNumber("elevator/pidPUP", Constants.Gains.ELEVATOR_UP_KP), 0)
//        talon.config_kI(0, SmartDashboard.getNumber("elevator/pidIUP", Constants.Gains.ELEVATOR_UP_KI), 0)
//        talon.config_kD(0, SmartDashboard.getNumber("elevator/pidDUP", Constants.Gains.ELEVATOR_UP_KD), 0)
//        talon.config_kF(0, SmartDashboard.getNumber("elevator/pidFUP", Constants.Gains.ELEVATOR_UP_KF), 0)

    }

    private fun setElevatorPosition(position: ElevatorState) {
//        var target = position.targetPos
//        if (target == Double.NaN) {
//            target = observedElevatorPosition
//        } else {
//            observedElevatorPosition = target
//        }
//        talon.set(ControlMode.MotionMagic, ElevatorConversion.inchesToPulses(target).toDouble())
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            elevatorState = ElevatorState.LOW
        }

        override fun onLoop() {
            synchronized(this@Elevator) {
                observedElevatorVelocity = -ElevatorConversion.nativeSpeedToInchesPerSecond(talon.sensorCollection.quadratureVelocity.toDouble())
                observedElevatorPosition = -ElevatorConversion.pulsesToInches(talon.sensorCollection.quadraturePosition.toDouble())
                elevatorPower = -talon.motorOutputPercent

                println("elevatorPos: $observedElevatorPosition")

                when (elevatorState){
                    ElevatorState.OPEN_LOOP -> {
                        return
                    }
                    ElevatorState.VELOCITY_CONTROL -> {
                        return
                    }
//                    ElevatorState.HIGH -> {
//                        setElevatorPosition(ElevatorState.HIGH)
//                    }
//                    ElevatorState.LOW -> {
//                        setElevatorPosition(ElevatorState.LOW)
//                    }
//                    ElevatorState.MEDIUM -> {
//                        setElevatorPosition(ElevatorState.MEDIUM)
//                    }
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
        movementState = MovementState.STILL
        setElevatorVelocity(0.0)

    }

    companion object {
        val instance = Elevator()
    }

    override fun zeroSensors() {
        talon.sensorCollection.setQuadraturePosition(0, 0)
    }
}