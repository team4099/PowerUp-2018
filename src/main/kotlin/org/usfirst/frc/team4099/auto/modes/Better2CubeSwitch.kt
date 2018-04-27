package org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.DashboardConfigurator
import org.usfirst.frc.team4099.auto.actions.*
import org.usfirst.frc.team4099.robot.subsystems.Intake

class Better2CubeSwitch(private val startingPosition: DashboardConfigurator.StartingPosition, private val ownershipConfig: String, private val delay: Double) : AutoModeBase()  {
    private val intake = Intake.instance

    override fun routine() {
        runAction(WaitAction(delay))
        runAction(DropWristAction())
        if (startingPosition == DashboardConfigurator.StartingPosition.CENTER) {
            //TODO: test changes; probably doesn't work as is
            runAction(ForwardDistanceAction(7.0))
            runAction(MoveElevatorAction(0.75))

            var dir = -1
            if (ownershipConfig[0] == 'L') {
                println("LEFT")
                runAction(TurnAction(-37.0))
                dir = 1
            } else {
                runAction(TurnAction(35.0))
            }
            runAction(ForwardDistanceAction(30.0))
            runAction(ForwardDistanceAction(30.0, true, false))

//            runAction(TurnAction(15.0 * dir, true, true))
//            runAction(ForwardDistanceAction(5.0))
            //            runAction(PushCubeOutAction())
            runAction(PushCubeOutAction())
//            runAction(WaitAction(0.5))
            runAction(ForwardDistanceAction(-5.0, slowMode = true, resetGyro = false))
            runAction(MoveElevatorAction(-1.0))

            // Come back
            runAction(TurnAction(-15.0 * dir))
            runAction(ForwardDistanceAction(-15.0))
            if (ownershipConfig[0] == 'L') {
                println("LEFT")
                runAction(TurnAction(37.0))
            } else {
                runAction(TurnAction(-35.0))
            }
            runAction(ForwardDistanceAction(-15.0))

            // Second Cube:
            runAction(OpenIntakeAction())
            runAction(ForwardUntilCubeAction(2.0, true))
            runAction(StopIntakeAction())
            runAction(ForwardDistanceAction(-30.0))
            SingleCubeSwitch(startingPosition, ownershipConfig, 0.0).routine()
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