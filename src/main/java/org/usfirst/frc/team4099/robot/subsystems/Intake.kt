package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop
import edu.wpi.first.wpilibj.DoubleSolenoid


/**
 * @author Team 4099
 *
 * This class is the constructor for the Intake subsystem
 *
 * @constructor Creates the Intake subsystem
 *
 */
class Intake private constructor() : Subsystem {

    private val rightTalon = TalonSRX(Constants.Intake.RIGHT_INTAKE_TALON_ID)
    private val leftTalon = TalonSRX(Constants.Intake.LEFT_INTAKE_TALON_ID)
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

    /**
     * stops intake
     */
    @Synchronized override fun stop() {
        intakeState = IntakeState.STOP
        setIntakePower(0.0)
    }

    /**
     * sets rightTalon to positive power and leftTalon to negative power
     * @param power a double that is the power for the intake
     */
    private fun setIntakePower(power: Double) {
        rightTalon.set(ControlMode.PercentOutput, -power)
        leftTalon.set(ControlMode.PercentOutput, power)
    }

    /**
     * Handles power to intake during each loop cycle
     * @constructor Creates loop that controls power to intake during each loop cycle
     */
    val loop: Loop = object : Loop {
        override fun onStart() {
            open = true
            intakeState = IntakeState.IN
        }

        /**
         * Sets Intake to -1 if pulling in, to 0 if stationary, and 1 if pushing out
         */
        override fun onLoop() {
            synchronized(this@Intake) {
                if (rightTalon.outputCurrent > 30 || leftTalon.outputCurrent > 30) {
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