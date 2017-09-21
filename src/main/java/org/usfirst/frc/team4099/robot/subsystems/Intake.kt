package org.usfirst.frc.team4099.robot.subsystems

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Intake private constructor() : Subsystem {

    private val upAndDown: DoubleSolenoid = DoubleSolenoid(
            Constants.Intake.UP_DOWN_SOLENOID_FORWARD,
            Constants.Intake.UP_DOWN_SOLENOID_REVERSE)
    private val gearGrabber: DoubleSolenoid = DoubleSolenoid(
            Constants.Intake.GRAB_SOLENOID_FORWARD,
            Constants.Intake.GRAB_SOLENOID_REVERSE)

    private var lastToggleIntake: Boolean = false

    val compressor: Compressor = Compressor()

    private var upTime: Double = 0.toDouble()

    enum class IntakePosition {
        UP_AND_CLOSED, UP_AND_OPEN, DOWN_AND_OPEN, DOWN_AND_CLOSED
    }

    @get:Synchronized
    var intakePosition: IntakePosition? = null
        private set

    init {

        intakePosition = IntakePosition.UP_AND_CLOSED
        upTime = -100.0
    }

    fun stopCompressor() {
        compressor.stop()
    }

    fun startCompressor() {
        compressor.start()
    }

    override fun outputToSmartDashboard() {
        if (intakePosition != null) {
            SmartDashboard.putBoolean("Intake.isUp", intakePosition != IntakePosition.DOWN_AND_OPEN)
            SmartDashboard.putBoolean("Intake.isClosed", intakePosition == IntakePosition.UP_AND_CLOSED)
        }
        SmartDashboard.putNumber("Compressor Current Draw", compressor.compressorCurrent)
        SmartDashboard.putBoolean("Pressure Switch Value", compressor.pressureSwitchValue)
    }

    @Synchronized override fun stop() {
        intakePosition = IntakePosition.UP_AND_CLOSED
        setIntakePositions()
    }

    override fun zeroSensors() {}

    @Synchronized
    fun updateIntake(toggleIntake: Boolean) {
        if (toggleIntake && !lastToggleIntake) {
            if (intakePosition == IntakePosition.DOWN_AND_OPEN) {
                this.upTime = Timer.getFPGATimestamp()
                intakePosition = IntakePosition.UP_AND_CLOSED
            } else if (intakePosition == IntakePosition.UP_AND_CLOSED) {
                intakePosition = IntakePosition.UP_AND_OPEN
            } else if (intakePosition == IntakePosition.UP_AND_OPEN) {
                intakePosition = IntakePosition.DOWN_AND_OPEN
            } else if (intakePosition == IntakePosition.DOWN_AND_CLOSED) {
                intakePosition = IntakePosition.UP_AND_CLOSED
            }
        }

        lastToggleIntake = toggleIntake
    }

    @Synchronized
    fun updateIntake(intakePosition: IntakePosition) {
        this.intakePosition = intakePosition
        setIntakePositions()
    }

    @Synchronized private fun setIntakePositions() {
        when (intakePosition) {
            Intake.IntakePosition.UP_AND_CLOSED -> {
                if (upTime == -1.0) {
                    upTime = Timer.getFPGATimestamp()
                }
                gearGrabber.set(DoubleSolenoid.Value.kReverse)
                if (Timer.getFPGATimestamp() - upTime > .4) {
                    upAndDown.set(DoubleSolenoid.Value.kReverse)
                }
            }
            Intake.IntakePosition.UP_AND_OPEN -> {
                gearGrabber.set(DoubleSolenoid.Value.kForward)
                upAndDown.set(DoubleSolenoid.Value.kReverse)
                upTime = -1.0
            }
            Intake.IntakePosition.DOWN_AND_OPEN -> {
                upAndDown.set(DoubleSolenoid.Value.kForward)
                gearGrabber.set(DoubleSolenoid.Value.kForward)
                upTime = -1.0
            }
            Intake.IntakePosition.DOWN_AND_CLOSED -> {
                upAndDown.set(DoubleSolenoid.Value.kForward)
                gearGrabber.set(DoubleSolenoid.Value.kReverse)
                upTime = -1.0
            }
        }
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            updateIntake(IntakePosition.UP_AND_CLOSED)
        }

        override fun onLoop() {
            synchronized(this@Intake) {
                setIntakePositions()
            }
        }

        override fun onStop() {
            stop()
        }
    }

    companion object {
        val instance = Intake()
    }

}