package org.usfirst.frc.team4099.auto.modes

// go to the baseline

import org.usfirst.frc.team4099.auto.AutoModeEndedException
import org.usfirst.frc.team4099.auto.actions.ForwardAction
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters

/**
 * Created by plato2000 on 2/13/17.
 */
class BaselineMode(initParameters: AutonomousInitParameters, private val turnAround: Boolean) : AutoModeBase() {
    private val initialForwardDistance: Double
    private val turningAngle: Double

    init {
        this.initialForwardDistance = initParameters.initalForwardSeconds
        this.turningAngle = initParameters.turnAngle.degrees
    }

    @Throws(AutoModeEndedException::class)
    override fun routine() {
        runAction(ForwardAction(initialForwardDistance))
        //        runAction(new TurnAction(Rotation2D.fromDegrees(60)));
    }


}
