package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.CANTalon
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Drive private constructor() : Subsystem {

    private val leftMasterSRX: CANTalon = CANTalon(Constants.Drive.LEFT_MASTER_ID)
    private val leftSlave1SRX: CANTalon = CANTalon(Constants.Drive.LEFT_SLAVE_1_ID)
    private val leftSlave2SRX: CANTalon = CANTalon(Constants.Drive.LEFT_SLAVE_2_ID)
    private val rightMasterSRX: CANTalon = CANTalon(Constants.Drive.RIGHT_MASTER_ID)
    private val rightSlave1SRX: CANTalon = CANTalon(Constants.Drive.RIGHT_SLAVE_1_ID)
    private val rightSlave2SRX: CANTalon = CANTalon(Constants.Drive.RIGHT_SLAVE_2_ID)
    private val ahrs: AHRS

    enum class DriveControlState {
        OPEN_LOOP
    }

    private var currentState = DriveControlState.OPEN_LOOP

    init {

        leftSlave1SRX.changeControlMode(CANTalon.TalonControlMode.Follower)
        leftSlave1SRX.set(Constants.Drive.LEFT_MASTER_ID.toDouble())
        leftSlave2SRX.changeControlMode(CANTalon.TalonControlMode.Follower)
        leftSlave2SRX.set(Constants.Drive.LEFT_MASTER_ID.toDouble())
        leftMasterSRX.changeControlMode(CANTalon.TalonControlMode.PercentVbus)

        rightSlave1SRX.changeControlMode(CANTalon.TalonControlMode.Follower)
        rightSlave1SRX.set(Constants.Drive.RIGHT_MASTER_ID.toDouble())
        rightSlave2SRX.changeControlMode(CANTalon.TalonControlMode.Follower)
        rightSlave2SRX.set(Constants.Drive.RIGHT_MASTER_ID.toDouble())
        rightMasterSRX.changeControlMode(CANTalon.TalonControlMode.PercentVbus)


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
        SmartDashboard.putNumber("leftTalon", leftMasterSRX.get())
        SmartDashboard.putNumber("rightTalon", rightMasterSRX.get())
    }

    fun startLiveWindowMode() {
        LiveWindow.addSensor("Drive", "Gyro", ahrs);
    }

    fun stopLiveWindowMode() {

    }

    fun updateLiveWindowTables() {

    }

    @Synchronized override fun stop() {
        setOpenLoop(DriveSignal.NEUTRAL)
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
        leftMasterSRX.set(-left)
        rightMasterSRX.set(right)
    }

    @Synchronized
    fun setOpenLoop(signal: DriveSignal) {
        if (currentState != DriveControlState.OPEN_LOOP) {
            currentState = DriveControlState.OPEN_LOOP
        }

        setLeftRightPower(signal.leftMotor, signal.rightMotor)
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
                    else -> {}
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
