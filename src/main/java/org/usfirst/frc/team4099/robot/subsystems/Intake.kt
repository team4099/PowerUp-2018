package org.usfirst.frc.team4099.robot.subsystems

import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.Talon
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.BrownoutDefender
import org.usfirst.frc.team4099.robot.loops.Loop

class Intake private constructor() : Subsystem {

    private val rightTalon = Talon(Constants.Intake.RIGHT_INTAKE_TALON_ID)
    private val leftTalon = Talon(Constants.Intake.LEFT_INTAKE_TALON_ID)
    private val pneumaticShifter: DoubleSolenoid = DoubleSolenoid(Constants.Intake.SHIFTER_FORWARD_ID,
            Constants.Intake.SHIFTER_REVERSE_ID)

    var intakeState = IntakeState.IN
    private var intakePower = 0.0
    var open: Boolean = false
        set (wantsOpen) {
            pneumaticShifter.set(if (wantsOpen) DoubleSolenoid.Value.kReverse else DoubleSolenoid.Value.kForward)
            field = wantsOpen
        }

    enum class IntakeState {
        IN, STOP, OUT, SLOW
    }

    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("intakePower", intakePower)
    }

    @Synchronized override fun stop() {
        intakeState = IntakeState.STOP
        setIntakePower(0.0)
    }

    private fun setIntakePower(power: Double) {
        rightTalon.set(-power)
        leftTalon.set(power)
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            open = true
            intakeState = IntakeState.IN
        }

        override fun onLoop() {
            synchronized(this@Intake) {
                if (BrownoutDefender.instance.getCurrent(Constants.Intake.LEFT_INTAKE_TALON_ID) > 30 || BrownoutDefender.instance.getCurrent(Constants.Intake.RIGHT_INTAKE_TALON_ID) > 30) {
                    intakeState = IntakeState.SLOW
                }
                when (intakeState) {
                    IntakeState.IN -> setIntakePower(-0.7)
                    IntakeState.STOP -> setIntakePower(0.0)
                    IntakeState.OUT -> setIntakePower(0.7)
                    IntakeState.SLOW -> setIntakePower(0.1)
                }
            }
        }

        override fun onStop() = stop()

    }

    companion object {
        val instance = Intake()
    }

    override fun zeroSensors() { }
}