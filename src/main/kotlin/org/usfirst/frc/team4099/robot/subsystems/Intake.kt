package org.usfirst.frc.team4099.robot.subsystems

import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DoubleSolenoid
import edu.wpi.first.wpilibj.Talon
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.BrownoutDefender
import org.usfirst.frc.team4099.robot.loops.Loop


/**
 * @author Team 4099
 *
 * This class is the constructor for the Intake subsystem
 *
 * @constructor Creates the Intake subsystem
 *
 */
class Intake private constructor() : Subsystem {

    private val rightTalon = Talon(Constants.Intake.RIGHT_INTAKE_TALON_ID)
    private val leftTalon = Talon(Constants.Intake.LEFT_INTAKE_TALON_ID)
    private val pneumaticShifter: DoubleSolenoid = DoubleSolenoid(Constants.Intake.SHIFTER_FORWARD_ID,
            Constants.Intake.SHIFTER_REVERSE_ID)
    private val ribbonSwitch = DigitalInput(Constants.Intake.RIBBON_SWITCH_PORT)

    var intakeState = IntakeState.IN
    private var intakePower = 0.0
    var open: Boolean = false
        set (wantsOpen) {
            pneumaticShifter.set(if (wantsOpen) DoubleSolenoid.Value.kReverse else DoubleSolenoid.Value.kForward)
            field = wantsOpen
        }

    enum class IntakeState {
        IN, STOP, SLOW_OUT, FAST_OUT, SLOW
    }

    override fun outputToSmartDashboard() {
        SmartDashboard.putNumber("intake/intakePower", intakePower)
        SmartDashboard.putBoolean("intake/isOpen", open)
        SmartDashboard.putNumber("intake/current", BrownoutDefender.instance.getCurrent(7))
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
        rightTalon.set(-power)
        leftTalon.set(power)
    }

    /**
     * Handles power to intake during each loop cycle
     * @constructor Creates loop that controls power to intake during each loop cycle
     */
    val loop: Loop = object : Loop {
        override fun onStart() {
            open = false
            intakeState = IntakeState.STOP
        }

        /**
         * Sets Intake to -1 if pulling in, to 0 if stationary, and 1 if pushing out
         */
        override fun onLoop() {
            synchronized(this@Intake) {
                if (ribbonSwitch.get() && (intakeState == IntakeState.IN || intakeState == IntakeState.SLOW)) {
//                if (intakeState == IntakeState.IN && (
//                                BrownoutDefender.instance.getCurrent(11) > 10
//                                || BrownoutDefender.instance.getCurrent(7) > 10)) {
                    intakeState = IntakeState.SLOW
                    open = false
                }
                when (intakeState) {
                    IntakeState.IN -> setIntakePower(-0.7)
                    IntakeState.STOP -> setIntakePower(0.0)
                    IntakeState.SLOW_OUT -> setIntakePower(0.5)
                    IntakeState.FAST_OUT -> setIntakePower(1.0)
                    IntakeState.SLOW -> setIntakePower(-0.5)
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