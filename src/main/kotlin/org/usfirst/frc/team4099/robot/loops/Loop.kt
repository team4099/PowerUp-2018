package org.usfirst.frc.team4099.robot.loops

interface Loop {
    fun onStart(timestamp: Double)
    fun onLoop(timestamp: Double)
    fun onStop(timestamp: Double)
}
