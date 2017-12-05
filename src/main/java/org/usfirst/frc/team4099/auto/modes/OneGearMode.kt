package org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.auto.AutoModeEndedException
import org.usfirst.frc.team4099.auto.actions.ForwardAction
import org.usfirst.frc.team4099.auto.actions.SetIntakeAction
import org.usfirst.frc.team4099.auto.actions.WaitAction
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters
import org.usfirst.frc.team4099.lib.util.Rotation2D
import org.usfirst.frc.team4099.robot.subsystems.Intake

/**
 * Created by plato2000 on 2/13/17.
 */
open class OneGearMode(initParameters: AutonomousInitParameters, private val backOut: Boolean, private val turnAround: Boolean) : AutoModeBase() {

    private val initialForwardSeconds: Double = initParameters.initalForwardSeconds
    private val initialTurn: Rotation2D = initParameters.turnAngle


    @Throws(AutoModeEndedException::class)
    override fun routine() {
        // If in center lane (already lined up with peg - don't use vision here
        // TODO: check if using vision is necessary in center lane
        println("initialTurn " + initialTurn.degrees)
        runAction(ForwardAction(initialForwardSeconds, true, true))
        runAction(SetIntakeAction(Intake.IntakePosition.DOWN_AND_OPEN))
        runAction(WaitAction(.75))
        runAction(ForwardAction(-1.0))
    }
}
