package org.usfirst.frc.team4099.robot.subsystems
import com.ctre.phoenix.motorcontrol.ControlMode
import org.usfirst.frc.team4099.robot.Constants

import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.robot.loops.Loop


class ramp private constructor() : Subsystem {
    private val pneumaticcylinder1: Solenoid = Solenoid(Constants.Drive.CYLINDER_MODULE,Constants.Drive.CYLINDER_CHANNEL)
    private val pneumaticcylinder2: Solenoid = Solenoid(Constants.Drive.CYLINDER_MODULE,Constants.Drive.CYLINDER_CHANNEL)
    private val pneumaticcylinder3: Solenoid = Solenoid(Constants.Drive.CYLINDER_MODULE,Constants.Drive.CYLINDER_CHANNEL)
    private val pneumaticcylinder4: Solenoid = Solenoid(Constants.Drive.CYLINDER_MODULE,Constants.Drive.CYLINDER_CHANNEL)


}
init{
    pneumaticcylinder1.set(DoubleSolenoid.Value.kForward)
    pneumaticcylinder2.set(DoubleSolenoid.Value.kForward)
    pneumaticcylinder3.set(DoubleSolenoid.Value.kForward)
    pneumaticcylinder4.set(DoubleSolenoid.Value.kForward)

}



