package org.usfirst.frc.team4099.robot.subsystems

import org.usfirst.frc.team4099.robot.loops.Loop

class Superstructure private constructor(): Subsystem {

    private val arm = Arm.instance
    //private val elevator = Elevator.instance
    private val drive = Drive.instance
    private val intake = Intake.instance
    //private val wrist = Wrist.instance


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
            TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
        }
        override fun onLoop() {
            TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
        }
        override fun onStop() {
            TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
        }
    }

    companion object {
        val instance = Superstructure()
    }


}