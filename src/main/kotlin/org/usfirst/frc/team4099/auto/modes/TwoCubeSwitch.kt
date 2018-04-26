package org.usfirst.frc.team4099.auto.modes

import edu.wpi.first.wpilibj.Timer
import org.usfirst.frc.team4099.DashboardConfigurator
import org.usfirst.frc.team4099.auto.actions.*
import org.usfirst.frc.team4099.robot.subsystems.Intake

class TwoCubeSwitch(private val startingPosition: DashboardConfigurator.StartingPosition, private val ownershipConfig: String, private val delay: Double) : AutoModeBase() {
    private val intake = Intake.instance

    override fun routine() {
        if (startingPosition == DashboardConfigurator.StartingPosition.CENTER ||
                startingPosition == DashboardConfigurator.StartingPosition.LEFT && ownershipConfig[0] == 'L' ||
                startingPosition == DashboardConfigurator.StartingPosition.RIGHT && ownershipConfig[0] == 'R') {
            SingleCubeSwitch(startingPosition, ownershipConfig, delay).routine()
            var turn = 90.0
            if (ownershipConfig[0] == 'L') {
                turn = -90.0
            }
            runAction(TurnAction(turn))
            runAction(ShiftGearAction(false))
            val startTime = Timer.getFPGATimestamp()
            runAction(ForwardUntilCubeAction(3.0))
            val elapsed = Timer.getFPGATimestamp() - startTime
            runAction(StopIntakeAction())
            if (intake.switchPressed) {
                runAction(WaitAction(0.5))
                runAction(ForwardAction(elapsed, -1))
                runAction(TurnAction(-turn))
                runAction(MoveElevatorAction(0.75))
                runAction(ForwardDistanceAction(45.0))
                runAction(OpenIntakeAction())
                runAction(WaitAction(2.0))
                runAction(ForwardDistanceAction(-30.0))
                runAction(MoveElevatorAction(-1.25))
            } else {
                runAction(ForwardDistanceAction(-15.0))
            }
            runAction(ShiftGearAction(true))
        }
        else {
            LineCrossMode(startingPosition, ownershipConfig, delay).routine()
        }
    }
}