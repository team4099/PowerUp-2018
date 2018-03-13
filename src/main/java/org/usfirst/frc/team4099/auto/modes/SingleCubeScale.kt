package org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.DashboardConfigurator
import org.usfirst.frc.team4099.auto.actions.*
import org.usfirst.frc.team4099.robot.subsystems.Elevator

class SingleCubeScale(private val startingPosition: DashboardConfigurator.StartingPosition, private val ownershipConfig: String, private val delay: Double) : AutoModeBase() {
    override fun routine() {
        runAction(WaitAction(delay))
        if (startingPosition == DashboardConfigurator.StartingPosition.CENTER) {
            runAction(ForwardDistanceAction(120.0))
        } else {
            runAction(ForwardDistanceAction(200.0))
            if ((startingPosition == DashboardConfigurator.StartingPosition.LEFT && ownershipConfig[0] == 'L')
                || (startingPosition == DashboardConfigurator.StartingPosition.RIGHT && ownershipConfig[0] == 'R')) {
                runAction(ParallelAction(arrayListOf(DropWristAction(), MoveElevatorAction(Elevator.ElevatorState.HIGH))))
                runAction(ForwardDistanceAction(30.0, true, true))
                runAction(PushCubeOutAction())
            } else {
                if (startingPosition == DashboardConfigurator.StartingPosition.RIGHT) {
                    runAction(TurnAction(-90.0))
                } else {
                    runAction(TurnAction(90.0))
                }
                runAction(ForwardDistanceAction(150.0))
                if (startingPosition == DashboardConfigurator.StartingPosition.RIGHT) {
                    runAction(TurnAction(90.0))
                } else {
                    runAction(TurnAction(-90.0))
                }
                runAction(ParallelAction(arrayListOf(DropWristAction(), MoveElevatorAction(Elevator.ElevatorState.HIGH))))
                runAction(ForwardDistanceAction(5.0))
                runAction(PushCubeOutAction())
            }
        }
    }
}