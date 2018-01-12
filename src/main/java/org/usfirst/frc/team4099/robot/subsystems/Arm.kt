package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.CANTalon

class Arm private constructor() : Subsystem {
    private val masterSRX: CANTalon = CANTalon(Constants.Arm.)
    override fun outputToSmartDashboard() {

        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun stop() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun zeroSensors() {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
    enum class ArmState {
        UP, STATIONARY, DOWN
    }
}