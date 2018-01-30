package org.usfirst.frc.team4099.robot.loops

import edu.wpi.first.wpilibj.DriverStation

/** Constantly measures battery voltage before the match begins.
 *
 */

class VoltageEstimator : Loop {

    @get:Synchronized
    var averageVoltage = 12.0
        private set
    private val weight = 15.0

    override fun onStart(timestamp: Double) {
        println("Robot disabled: computing avg voltage")
    }

    @Synchronized override fun onLoop(timestamp: Double) {
        val cur_voltage = DriverStation.getInstance().batteryVoltage
        averageVoltage = (cur_voltage + weight * averageVoltage) / (1.0 + weight)
    }

    override fun onStop(timestamp: Double) {
        println("Robot enabled: last avg voltage: " + averageVoltage)
    }

    companion object {

        val instance = VoltageEstimator()
    }
}
