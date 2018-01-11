package org.usfirst.frc.team4099.robot.subsystems


import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.*

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop
import edu.wpi.first.wpilibj.DriverStation





class Drive private constructor() : Subsystem {

    private val leftMasterSRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_MASTER_ID)
    private val leftSlave1SRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_SLAVE_1_ID)
    private val leftSlave2SRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_SLAVE_2_ID)
    private val rightMasterSRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_MASTER_ID)
    private val rightSlave1SRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_SLAVE_1_ID)
    private val rightSlave2SRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_SLAVE_2_ID)
    private val ahrs: AHRS
    private var brakeMode: NeutralMode=NeutralMode.Brake//sets whether the break mode should be coast (no resistence) or by force

    enum class DriveControlState {
        OPEN_LOOP
    }

    private var currentState = DriveControlState.OPEN_LOOP

    init {
        leftSlave1SRX.set(ControlMode.Follower, Constants.Drive.LEFT_MASTER_ID.toDouble())
        leftSlave2SRX.set(ControlMode.Follower, Constants.Drive.LEFT_MASTER_ID.toDouble())
        leftMasterSRX.set(ControlMode.PercentOutput, 0.0)
        leftMasterSRX.setSensorPhase(true)
        leftMasterSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)
        leftMasterSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10)
        leftMasterSRX.config_kP(0, Constants.Gains.LEFT_KP, 10)
        leftMasterSRX.config_kI(0, Constants.Gains.LEFT_KI, 10)
        leftMasterSRX.config_kD(0, Constants.Gains.LEFT_KD, 10)
        leftMasterSRX.config_kF(0, Constants.Gains.LEFT_KF, 10)
        /*val leftSensorPresent = leftMasterSRX.isSensorPresent(FeedbackDevice.CTRE_MagEncoder_Relative)
        if (leftSensorPresent !== TalonSRX.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left encoder: " + leftSensorPresent, false)
        }*/


        rightSlave1SRX.set(ControlMode.Follower, Constants.Drive.RIGHT_MASTER_ID.toDouble())
        rightSlave2SRX.set(ControlMode.Follower, Constants.Drive.RIGHT_MASTER_ID.toDouble())
        rightMasterSRX.set(ControlMode.PercentOutput, 0.0)
        rightMasterSRX.setSensorPhase(true)
        rightMasterSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)
        rightMasterSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10)
        rightMasterSRX.config_kP(0, Constants.Gains.RIGHT_KP, 10)
        rightMasterSRX.config_kI(0, Constants.Gains.RIGHT_KI, 10)
        rightMasterSRX.config_kD(0, Constants.Gains.RIGHT_KD, 10)
        rightMasterSRX.config_kF(0, Constants.Gains.RIGHT_KF, 10)

        leftMasterSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms,10)
        leftMasterSRX.configVelocityMeasurementWindow(32, 10)
        rightMasterSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms,10)
        rightMasterSRX.configVelocityMeasurementWindow(32,10)

        setOpenLoop(DriveSignal.NEUTRAL)



        ahrs = AHRS(SPI.Port.kMXP)

        this.zeroSensors()
    }

    fun getBrakeMode(): NeutralMode{
        return brakeMode
    }
    fun setBrakeMode(type: NeutralMode){
        if (brakeMode!=type) {
            brakeMode = type
            rightMasterSRX.setNeutralMode(type)
            rightSlave1SRX.setNeutralMode(type)
            rightSlave2SRX.setNeutralMode(type)
            leftMasterSRX.setNeutralMode(type)
            leftSlave1SRX.setNeutralMode(type)
            leftSlave2SRX.setNeutralMode(type)
        }
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
                    DriveControlState.OPEN_LOOP -> {
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
