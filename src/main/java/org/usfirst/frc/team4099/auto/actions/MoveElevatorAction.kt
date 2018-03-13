package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer
import org.usfirst.frc.team4099.robot.subsystems.Elevator

/**
 * Created by Oksana on 2/16/2017.
 */
class MoveElevatorAction(private val newElevatorState: Elevator.ElevatorState) : Action {
    private val mElevator = Elevator.instance
    private var startTime = 0.0

    override fun isFinished(): Boolean {
        return mElevator.movementState == Elevator.MovementState.STILL && Timer.getFPGATimestamp() - startTime > 0.5
    }

    override fun update() {

    }

    override fun done() {

    }

    override fun start() {
        startTime = Timer.getFPGATimestamp()
        mElevator.elevatorState = newElevatorState
    }
}
