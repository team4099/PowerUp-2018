package org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.DashboardConfigurator
import org.usfirst.frc.team4099.auto.actions.*

class SingleCubeSwitch(private val startingPosition: DashboardConfigurator.StartingPosition, private val ownershipConfig: String, private val delay: Double) : AutoModeBase() {
    override fun routine() {
        runAction(WaitAction(delay))
        runAction(DropWristAction())
        if (startingPosition == DashboardConfigurator.StartingPosition.CENTER) {
            runAction(ForwardDistanceAction(15.0))
            runAction(MoveElevatorAction(1.5))

            if (ownershipConfig[0] == 'L') {
                println("LEFT")
                runAction(TurnAction(-35.0))
            } else {
                runAction(TurnAction(35.0))
            }
            runAction(ForwardDistanceAction(90.0))
            runAction(ForwardDistanceAction(20.0))
//            runAction(PushCubeOutAction())
            runAction(OpenIntakeAction())
            runAction(WaitAction(2.0))
            runAction(ForwardDistanceAction(-45.0))
            runAction(MoveElevatorAction(-2.0))
        } else {
            runAction(ForwardDistanceAction(60.0))
            if ((startingPosition == DashboardConfigurator.StartingPosition.LEFT && ownershipConfig[0] == 'L')
                || (startingPosition == DashboardConfigurator.StartingPosition.RIGHT && ownershipConfig[0] == 'R')) {
                runAction(MoveElevatorAction(1.0))
                runAction(ForwardDistanceAction(40.0))
//                runAction(PushCubeOutAction())
                runAction(OpenIntakeAction())
                runAction(WaitAction(2.0))
                runAction(ForwardDistanceAction(-45.0))
                runAction(MoveElevatorAction(-2.0))
            }
        }
    }
}