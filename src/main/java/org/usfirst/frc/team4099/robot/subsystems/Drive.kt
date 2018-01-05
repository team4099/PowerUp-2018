package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Drive private constructor() : Subsystem {

    private val leftMasterSRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_MASTER_ID)
    private val leftSlave1SRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_SLAVE_1_ID)
    private val leftSlave2SRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_SLAVE_2_ID)
    private val rightMasterSRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_MASTER_ID)
    private val rightSlave1SRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_SLAVE_1_ID)
    private val rightSlave2SRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_SLAVE_2_ID)
    private val ahrs: AHRS

    enum class DriveControlState {
        OPEN_LOOP
    }

    private var currentState = DriveControlState.OPEN_LOOP

    init {
        leftSlave1SRX.set(ControlMode.Follower, Constants.Drive.LEFT_MASTER_ID.toDouble())
        leftSlave2SRX.set(ControlMode.Follower, Constants.Drive.LEFT_MASTER_ID.toDouble())
        leftMasterSRX.set(ControlMode.PercentOutput, 0.0)

        rightSlave1SRX.set(ControlMode.Follower, Constants.Drive.RIGHT_MASTER_ID.toDouble())
        rightSlave2SRX.set(ControlMode.Follower, Constants.Drive.RIGHT_MASTER_ID.toDouble())
        rightMasterSRX.set(ControlMode.PercentOutput, 0.0)

        ahrs = AHRS(SPI.Port.kMXP)

        this.zeroSensors()
    }

    fun getAHRS(): AHRS? {
        return if (ahrs.isConnected) ahrs else null
    }

    override fun outputToSmartDashboard() {
        if (this.getAHRS() != null) {
            SmartDashboard.putNumber("gyro", this.getAHRS()!!.yaw.toDouble())
        } else {
            SmartDashboard.putNumber("gyro", -31337.0)
        }
        SmartDashboard.putNumber("leftTalon", leftMasterSRX.motorOutputVoltage)
        SmartDashboard.putNumber("rightTalon", rightMasterSRX.motorOutputVoltage)
    }

    fun startLiveWindowMode() {
        LiveWindow.addSensor("Drive", "Gyro", ahrs);
    }

    fun stopLiveWindowMode() {

    }

    fun updateLiveWindowTables() {

    }

    override fun zeroSensors() {
        if (ahrs.isConnected) {
            ahrs.reset()
        }
    }

    /**
     * Powers the left and right talons during OPEN_LOOP
     * @param left
     * @param right
     */
    @Synchronized private fun setLeftRightPower(left: Double, right: Double) {
        leftMasterSRX.set(ControlMode.PercentOutput, - left)
        rightMasterSRX.set(ControlMode.PercentOutput, right)
    }

    @Synchronized
    fun setOpenLoop(signal: DriveSignal) {
        if (currentState != DriveControlState.OPEN_LOOP) {
            currentState = DriveControlState.OPEN_LOOP
        }

        setLeftRightPower(signal.leftMotor, signal.rightMotor)
    }

    @Synchronized override fun stop() {
        setOpenLoop(DriveSignal.NEUTRAL)
    }

    val loop: Loop = object : Loop {
        override fun onStart() {
            setOpenLoop(DriveSignal.NEUTRAL)
        }

        override fun onLoop() {
            synchronized(this@Drive) {
                when (currentState) {
                    Drive.DriveControlState.OPEN_LOOP -> {
                    }
                    else -> {
                    }
                }
            }
        }

        override fun onStop() {
            setOpenLoop(DriveSignal.NEUTRAL)
        }
    }

    companion object {
        val instance = Drive()
    }

}
