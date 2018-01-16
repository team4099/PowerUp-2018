package org.usfirst.frc.team4099.robot.subsystems


import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.can.BaseMotorController
import com.ctre.phoenix.motorcontrol.ControlMode

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop
import org.usfirst.frc.team4099.robot.subsystems.Drive.DriveControlState
import com.ctre.CANTalon






class Drive private constructor() : Subsystem {

    private val leftMasterSRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_MASTER_ID)
    private val leftSlave1SRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_SLAVE_1_ID)
    private val leftSlave2SRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_SLAVE_2_ID)
    private val rightMasterSRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_MASTER_ID)
    private val rightSlave1SRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_SLAVE_1_ID)
    private val rightSlave2SRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_SLAVE_2_ID)
    private val ahrs: AHRS
    private var brakeMode: NeutralMode=NeutralMode.Brake//sets whether the break mode should be coast (no resistence) or by force
    private var highGear: Boolean=true

    enum class DriveControlState {
        OPEN_LOOP,
        VELOCITY_SETPOINT,
        PATH_FOLLOWING,
        TURN_TO_HEADING //turn in place
    }

    private var currentState = DriveControlState.OPEN_LOOP

    init {
        leftSlave1SRX.set(ControlMode.Follower, Constants.Drive.LEFT_MASTER_ID.toDouble())
        leftSlave2SRX.set(ControlMode.Follower, Constants.Drive.LEFT_MASTER_ID.toDouble())
        leftMasterSRX.set(ControlMode.PercentOutput, 0.0)
        leftMasterSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10)
        leftMasterSRX.setSensorPhase(false)
        leftMasterSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10)
        leftMasterSRX.config_kP(0, Constants.Gains.LEFT_LOW_KP, 10)
        leftMasterSRX.config_kI(0, Constants.Gains.LEFT_LOW_KI, 10)
        leftMasterSRX.config_kD(0, Constants.Gains.LEFT_LOW_KD, 10)
        leftMasterSRX.config_kF(0, Constants.Gains.LEFT_LOW_KF, 10)

        leftMasterSRX.config_kP(1, Constants.Gains.LEFT_HIGH_KP, 10)
        leftMasterSRX.config_kI(1, Constants.Gains.LEFT_HIGH_KI, 10)
        leftMasterSRX.config_kD(1, Constants.Gains.LEFT_HIGH_KD, 10)
        leftMasterSRX.config_kF(1, Constants.Gains.LEFT_HIGH_KF, 10)



        rightSlave1SRX.set(ControlMode.Follower, Constants.Drive.RIGHT_MASTER_ID.toDouble())
        rightSlave2SRX.set(ControlMode.Follower, Constants.Drive.RIGHT_MASTER_ID.toDouble())
        rightMasterSRX.set(ControlMode.PercentOutput, 0.0)
        rightMasterSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10)
        rightMasterSRX.setSensorPhase(false)
        rightMasterSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 10)
        rightMasterSRX.config_kP(0, Constants.Gains.RIGHT_LOW_KP, 10)
        rightMasterSRX.config_kI(0, Constants.Gains.RIGHT_LOW_KI, 10)
        rightMasterSRX.config_kD(0, Constants.Gains.RIGHT_LOW_KD, 10)
        rightMasterSRX.config_kF(0, Constants.Gains.RIGHT_LOW_KF, 10)

        rightMasterSRX.config_kP(1, Constants.Gains.RIGHT_HIGH_KP, 10)
        rightMasterSRX.config_kI(1, Constants.Gains.RIGHT_HIGH_KI, 10)
        rightMasterSRX.config_kD(1, Constants.Gains.RIGHT_HIGH_KD, 10)
        rightMasterSRX.config_kF(1, Constants.Gains.RIGHT_HIGH_KF, 10)

        leftMasterSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms,10)
        leftMasterSRX.configVelocityMeasurementWindow(32, 10)
        rightMasterSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms,10)
        rightMasterSRX.configVelocityMeasurementWindow(32,10)

        setOpenLoop(DriveSignal.NEUTRAL)



        ahrs = AHRS(SPI.Port.kMXP)

        this.zeroSensors()
    }

    fun setOpenLoop(signal: DriveSignal) {
        if (currentState !== DriveControlState.OPEN_LOOP) {
            leftMasterSRX.set(ControlMode.PercentOutput, 0.0)
            rightMasterSRX.set(ControlMode.PercentOutput, 0.0)
            leftMasterSRX.configNominalOutputForward(0.0, 10)
            rightMasterSRX.configNominalOutputForward(0.0, 10)
            currentState = DriveControlState.OPEN_LOOP
            setBrakeMode(NeutralMode.Brake)
        }
        setLeftRightPower(signal.leftMotor, signal.rightMotor)
    }




    fun onStart(timestamp: Double){
        synchronized (this) {
            setOpenLoop(DriveSignal.NEUTRAL)
            setBrakeMode(NeutralMode.Coast)
            setVelocitySetpoint(0, 0)
        }
    }

    fun usesTalonVelocityControl(state: DriveControlState): Boolean {
        if (state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING) {
            return true
        }
        return false
    }

    fun usesTalonPositionControl(state: DriveControlState): Boolean {
        if (state == DriveControlState.TURN_TO_HEADING) {
            return true
        }
        return false
    }


    fun getHighGear():Boolean {
        return highGear
    }

    fun setHighGear(choice: Boolean){
        highGear=choice
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

    override fun stop(){
        synchronized(this){
            setOpenLoop(DriveSignal.NEUTRAL)
        }
    }

    fun resetEncoders(){

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

    @Synchronized
    fun setVelocitySetpoint(left_inches_per_sec: Double, right_inches_per_sec: Double) {
        configureTalonsForVelocityControl()
        currentState = DriveControlState.VELOCITY_SETPOINT
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec)
    }

    private fun configureTalonsForVelocityControl() { //should further review cause im bad
        if (!usesTalonVelocityControl(currentState)) {
            // We entered a velocity control state.
            leftMasterSRX.set(ControlMode.Velocity,0.0) //velocity  output value is in position change / 100ms
            leftMasterSRX.configNominalOutputReverse(Constants.Velocity.driveHighGearNominalOutput, 10)
            leftMasterSRX.selectProfileSlot(Constants.Velocity.highGearVelocityControlSlot, 0)
            leftMasterSRX.configPeakOutputForward(Constants.Velocity.driveHighGearNominalOutput, 10)
            rightMasterSRX.set(ControlMode.Velocity,0.0) //velocity  output value is in position change / 100ms
            rightMasterSRX.configNominalOutputReverse(Constants.Velocity.driveHighGearNominalOutput, 10)
            rightMasterSRX.selectProfileSlot(Constants.Velocity.highGearVelocityControlSlot, 0)
            rightMasterSRX.configPeakOutputForward(Constants.Velocity.driveHighGearNominalOutput, 10)
            setBrakeMode(NeutralMode.Brake)
        }
    }
     private fun configureTalonsforPositionControl(){

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
