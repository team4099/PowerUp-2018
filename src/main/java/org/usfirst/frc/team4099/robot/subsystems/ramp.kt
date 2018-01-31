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
    private val leftSlave2SRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_SLAVE_2_ID)
    private val rightSlave2SRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_SLAVE_2_ID)



}

    leftSlave2SRX.set(ControlMode.Follower, Constants.Drive.LEFT_MASTER_ID.toDouble())
    rightSlave2SRX.set(ControlMode.Follower, Constants.Drive.RIGHT_MASTER_ID.toDouble())
    fun deploy(){
        rightMaster2SRX.set(ControlMode.PercentOutput, right)
        leftMaster2SRX.set(ControlMode.PercentOutput, right)




    }
    fun actuate(){
        pneumaticShifter1.set(DoubleSolenoid.Value.kForward)
        pneumaticShifter2.set(DoubleSolenoid.Value.kForward)
        pneumaticShifter3.set(DoubleSolenoid.Value.kForward)
        pneumaticShifter4.set(DoubleSolenoid.Value.kForward)

    }





