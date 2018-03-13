package org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.DashboardConfigurator
import org.usfirst.frc.team4099.auto.actions.*
import org.usfirst.frc.team4099.robot.subsystems.Elevator

class SingleCubeSwitch(private val startingPosition: DashboardConfigurator.StartingPosition, private val ownershipConfig: String, private val delay: Double) : AutoModeBase() {
    override fun routine() {
        runAction(WaitAction(delay))
        runAction(DropWristAction())
        if (startingPosition == DashboardConfigurator.StartingPosition.CENTER) {
            runAction(ForwardDistanceAction(25.0))
            if (ownershipConfig[0] == 'L') {
                runAction(TurnAction(25.0))
            } else {
                runAction(TurnAction(-25.0))
            }
            runAction(ForwardDistanceAction(80.0))
            runAction(MoveElevatorAction(Elevator.ElevatorState.MEDIUM))
            runAction(ForwardDistanceAction(20.0))
            runAction(PushCubeOutAction())
            runAction(OpenIntakeAction())
        } else {
            runAction(ForwardDistanceAction(90.0))
            if ((startingPosition == DashboardConfigurator.StartingPosition.LEFT && ownershipConfig[0] == 'L')
                || (startingPosition == DashboardConfigurator.StartingPosition.RIGHT && ownershipConfig[0] == 'R')) {
                runAction(MoveElevatorAction(Elevator.ElevatorState.MEDIUM))
                runAction(ForwardDistanceAction(30.0))
                runAction(PushCubeOutAction())
                runAction(OpenIntakeAction())
            }
        }
    }
}