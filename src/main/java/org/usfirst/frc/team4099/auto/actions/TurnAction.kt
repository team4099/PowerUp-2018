package org.usfirst.frc.team4099.auto.actions

import org.usfirst.frc.team4099.lib.util.Rotation2D
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.subsystems.Drive

/**
 * Created by plato2000 on 2/14/17.
 */
class TurnAction constructor(degreesToTurn: Rotation2D, absolutePosition: Boolean = false) : Action {

    private val mDrive: Drive = Drive.instance
    private var isDone: Boolean = false
    private var finishAngle: Double = 0.toDouble()

    init {
        this.isDone = false
        if (!absolutePosition) {
            finishAngle = degreesToTurn.degrees
        } else {
            finishAngle = (180 - mDrive.getAHRS()!!.yaw + degreesToTurn.degrees) % 360 - 180
        }
    }

    override fun isFinished(): Boolean {
        return isDone || Math.abs(mDrive.getAHRS()!!.yaw - finishAngle) < Constants.Drive.TURN_TOLERANCE_DEGREES
    }

    override fun update() {
        isDone = mDrive.turnAngle()
        //        System.out.println("Still turning "  + isDone);
    }

    override fun done() {
        mDrive.finishTurn()
    }

    override fun start() {
        mDrive.setAngleSetpoint(finishAngle)
    }
}
