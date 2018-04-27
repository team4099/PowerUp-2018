package org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.DashboardConfigurator
import org.usfirst.frc.team4099.auto.actions.*

class SingleCubeEdgeSwitch(private val startingPosition: DashboardConfigurator.StartingPosition, private val ownershipConfig: String, private val delay: Double) : AutoModeBase() {
    override fun routine() {
        runAction(WaitAction(delay))
        runAction(DropWristAction())
        if (startingPosition == DashboardConfigurator.StartingPosition.CENTER) {
            //TODO: test changes; probably doesn't work as is
            runAction(ForwardDistanceAction(15.0))
            runAction(MoveElevatorAction(0.75))

            var dir = -1
            if (ownershipConfig[0] == 'L') {
                println("LEFT")
                runAction(TurnAction(-45.0))
                dir = 1
            } else {
                runAction(TurnAction(35.0))
            }
            runAction(ForwardDistanceAction(100.0))

//            runAction(TurnAction(15.0 * dir))
//            runAction(PushCubeOutAction())
            runAction(OpenIntakeAction())
            runAction(WaitAction(2.0))
            runAction(ForwardDistanceAction(-30.0))
            runAction(MoveElevatorAction(-1.25))
        } else {
            runAction(ForwardDistanceAction(170.0))
            if ((startingPosition == DashboardConfigurator.StartingPosition.LEFT && ownershipConfig[0] == 'L')
                || (startingPosition == DashboardConfigurator.StartingPosition.RIGHT && ownershipConfig[0] == 'R')) {
                runAction(MoveElevatorAction(1.0))
                if(startingPosition == DashboardConfigurator.StartingPosition.LEFT) {
                    runAction(TurnAction(90.0))
                } else {
                    runAction(TurnAction(-90.0))
                }
                runAction(ForwardDistanceAction(20.0))
//                runAction(PushCubeOutAction())
                runAction(OpenIntakeAction())
                runAction(WaitAction(2.0))
                runAction(ForwardDistanceAction(-25.0))
                runAction(MoveElevatorAction(-2.0))
            }
        }
    }
}