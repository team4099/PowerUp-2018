package org.usfirst.frc.team4099.robot.subsystems

import org.usfirst.frc.team4099.robot.loops.Loop

class Superstructure private constructor(): Subsystem {

    private val arm = Arm.instance
    //private val elevator = Elevator.instance
    private val drive = Drive.instance
    private val intake = Intake.instance
    //private val wrist = Wrist.instance
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
            TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
        }
        override fun onLoop() {
            synchronized(this@Superstructure){
                when(arm.armState){
                    Arm.ArmState.LOW ->{
                        wrist.wristState = Wrist.WristState.STOWED_UP
                    }
                    Arm.ArmState.EXCHANGE ->{
                        wrist.wristState = Wrist.WristState.HORIZONTAL
                    }
                    Arm.ArmState.HIGH ->{
                        wrist.wristState = Wrist.WristState.STOWED_DOWN
                    }
                }
                when(systemState){
                    SystemState.IDLE -> {}
                    SystemState.AUTO -> {}
                    SystemState.TELEOP -> {}
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