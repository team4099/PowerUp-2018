package org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.DashboardConfigurator
import org.usfirst.frc.team4099.auto.actions.ForwardDistanceAction
import org.usfirst.frc.team4099.auto.actions.WaitAction

class LineCrossMode(private val startingPosition: DashboardConfigurator.StartingPosition, private val ownershipConfig: String, private val delay: Double) : AutoModeBase() {
    override fun routine() {
        runAction(WaitAction(delay))
        runAction(ForwardDistanceAction(120.0))
    }
}