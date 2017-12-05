package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.CANTalon
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Drive private constructor() : Subsystem {

    private val leftSRX: CANTalon = CANTalon(Constants.Drive.LEFT_FRONT_ID)
    private val leftSlaveSRX: CANTalon = CANTalon(Constants.Drive.LEFT_BACK_ID)
    private val rightSRX: CANTalon = CANTalon(Constants.Drive.RIGHT_FRONT_ID)
    private val rightSlaveSRX: CANTalon = CANTalon(Constants.Drive.RIGHT_BACK_ID)
    private val ahrs: AHRS

    enum class DriveControlState {
        OPEN_LOOP
    }

    private var currentState = DriveControlState.OPEN_LOOP

    init {

        leftSlaveSRX.changeControlMode(CANTalon.TalonControlMode.Follower)
        leftSlaveSRX.set(Constants.Drive.LEFT_FRONT_ID.toDouble())
        leftSRX.changeControlMode(CANTalon.TalonControlMode.PercentVbus)

        rightSlaveSRX.changeControlMode(CANTalon.TalonControlMode.Follower)
        rightSlaveSRX.set(Constants.Drive.RIGHT_FRONT_ID.toDouble())
        rightSRX.changeControlMode(CANTalon.TalonControlMode.PercentVbus)


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
        SmartDashboard.putNumber("leftTalon", leftSRX.get())
        SmartDashboard.putNumber("rightTalon", rightSRX.get())
    }

    fun startLiveWindowMode() {
        //        System.out.println("do a potato");
        //        LiveWindow.addActuator("Drive", "turnController", turnController);
        //        LiveWindow.addActuator("Drive", "leftController", leftController);
        //        LiveWindow.addActuator("Drive", "rightController", rightController);
        //        LiveWindow.addSensor("Drive", "Gyro", ahrs);
        //        LiveWindow.addSensor("Drive", "leftEncoder", leftEncoder);
        //        LiveWindow.addSensor("Drive", "rightEncoder", rightEncoder);
        //        leftEncoder.startLiveWindowMode();
        //        rightEncoder.startLiveWindowMode();
        //        leftController.startLiveWindowMode();
        //        rightController.startLiveWindowMode();

    }

    fun stopLiveWindowMode() {

    }

    fun updateLiveWindowTables() {
        //        leftEncoder.updateTable();
        //        rightEncoder.updateTable();
        //        leftController.updateTable();
        //        rightController.updateTable();
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
        //        System.out.println("Left: " + left + "Right: " + right);
        leftSRX.set(-left)
        rightSRX.set(right)
    }

    @Synchronized
    fun setOpenLoop(signal: DriveSignal) {
        if (currentState != DriveControlState.OPEN_LOOP) {
            currentState = DriveControlState.OPEN_LOOP
        }


        setLeftRightPower(signal.leftMotor, signal.rightMotor)
    }

    fun arcadeDrive(outputMagnitude: Double, curve: Double) {
        val leftOutput: Double
        val rightOutput: Double

        if (curve < 0) {
            val value = Math.log(-curve)
            var ratio = (value - .5) / (value + .5)
            if (ratio == 0.0) {
                ratio = .0000000001
            }
            leftOutput = outputMagnitude / ratio
            rightOutput = outputMagnitude
        } else if (curve > 0) {
            val value = Math.log(curve)
            var ratio = (value - .5) / (value + .5)
            if (ratio == 0.0) {
                ratio = .0000000001
            }
            leftOutput = outputMagnitude
            rightOutput = outputMagnitude / ratio
        } else {
            leftOutput = outputMagnitude
            rightOutput = outputMagnitude
        }
        setLeftRightPower(leftOutput, rightOutput)
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
