package org.usfirst.frc.team4099.robot.subsystems
import com.ctre.phoenix.motorcontrol.ControlMode
import org.usfirst.frc.team4099.robot.Constants

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.loops.Loop


class ramp private constructor() : Subsystem {
    private val pneumaticShifter1: Solenoid = Solenoid(Constants.Drive.SHIFTER_MODULE,Constants.Drive.SHIFTER_CHANNEL)
    private val pneumaticShifter2: Solenoid = Solenoid(Constants.Drive.SHIFTER_MODULE,Constants.Drive.SHIFTER_CHANNEL)
    private val pneumaticShifter3: Solenoid = Solenoid(Constants.Drive.SHIFTER_MODULE,Constants.Drive.SHIFTER_CHANNEL)
    private val pneumaticShifter4: Solenoid = Solenoid(Constants.Drive.SHIFTER_MODULE,Constants.Drive.SHIFTER_CHANNEL)


}


