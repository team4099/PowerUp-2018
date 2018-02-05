package org.usfirst.frc.team4099.robot.subsystems

import org.usfirst.frc.team4099.robot.loops.Loop
import org.usfirst.frc.team4099.robot.Constants

class Superstructure private constructor(): Subsystem {

    private val arm = Arm.instance
    private val elevator = Elevator.instance
    private val drive = Drive.instance
    private val intake = Intake.instance
    private val wrist = Wrist.instance
    enum class SystemState{
        IDLE, AUTO, TELEOP //not sure
    }

    var systemState = SystemState.IDLE


    override fun outputToSmartDashboard() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun stop() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    override fun zeroSensors(){

    }

    val loop : Loop = object : Loop {
        override fun onStart() {
            wrist.WristState = Wrist.WristState.HORIZONTAL
        }
        override fun onLoop() {
            if(Math.cos(arm.armAngle) * Constants.Superstructure.armLength - Constants.Superstructure.distToFront + Constants.Superstructure.intakeLength > Constants.Superstructure.maxExtendDist){
                if (arm.armAngle > Arm.ArmState.EXCHANGE.targetPos) {
                    wrist.wristState = Wrist.WristState.STOWED_DOWN
                } else {
                    wrist.wristState = Wrist.WristState.STOWED_UP
                }

            }
        }
        override fun onStop() {
            TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
        }
    }

    companion object {
        val instance = Superstructure()
    }


}