package org.usfirst.frc.team4099.robot.subsystems

import com.ctre.MotorControl.CANTalon
import com.ctre.MotorControl.SmartMotorController
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.lib.drive.PIDOutputReceiver
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

class Drive private constructor() : Subsystem {

    private val leftSRX: CANTalon = CANTalon(Constants.Drive.LEFT_FRONT_ID)
    private val leftSlaveSRX: CANTalon = CANTalon(Constants.Drive.LEFT_BACK_ID)
    private val rightSRX: CANTalon = CANTalon(Constants.Drive.RIGHT_FRONT_ID)
    private val rightSlaveSRX: CANTalon = CANTalon(Constants.Drive.RIGHT_BACK_ID)
    private val ahrs: AHRS

    enum class DriveControlState {
        OPEN_LOOP,
        AUTONOMOUS_TURNING,
        AUTONOMOUS_DRIVING
    }

    private var currentState = DriveControlState.OPEN_LOOP

    private val turnController: PIDController
    private val leftController: PIDController
    private val rightController: PIDController

    private val turnReceiver: PIDOutputReceiver
    private val leftReceiver: PIDOutputReceiver
    private val rightReceiver: PIDOutputReceiver

    private var leftEncoder: Encoder
    private var rightEncoder: Encoder

    init {

        leftSlaveSRX.changeControlMode(SmartMotorController.TalonControlMode.Follower)
        leftSlaveSRX.set(Constants.Drive.LEFT_FRONT_ID.toDouble())
        leftSRX.changeControlMode(SmartMotorController.TalonControlMode.PercentVbus)

        rightSlaveSRX.changeControlMode(SmartMotorController.TalonControlMode.Follower)
        rightSlaveSRX.set(Constants.Drive.RIGHT_FRONT_ID.toDouble())
        rightSRX.changeControlMode(SmartMotorController.TalonControlMode.PercentVbus)


        ahrs = AHRS(SPI.Port.kMXP)

        leftEncoder = Encoder(Constants.Drive.LEFT_ENCODER_A, Constants.Drive.LEFT_ENCODER_B, true, CounterBase.EncodingType.k4X)
        leftEncoder.samplesToAverage = Constants.Drive.ENCODER_SAMPLES_TO_AVERAGE
        leftEncoder.setDistancePerPulse(Constants.Drive.LEFT_ENCODER_INCHES_PER_PULSE)

        rightEncoder = Encoder(Constants.Drive.RIGHT_ENCODER_A, Constants.Drive.RIGHT_ENCODER_B, false, CounterBase.EncodingType.k4X)
        rightEncoder.samplesToAverage = Constants.Drive.ENCODER_SAMPLES_TO_AVERAGE
        rightEncoder.setDistancePerPulse(Constants.Drive.RIGHT_ENCODER_INCHES_PER_PULSE)

        turnReceiver = PIDOutputReceiver()
        turnController = PIDController(Constants.Gains.TURN_P, Constants.Gains.TURN_I, Constants.Gains.TURN_D, Constants.Gains.TURN_F, ahrs, turnReceiver)
        turnController.setInputRange(-180.0, 180.0)
        turnController.setOutputRange(-Constants.Drive.AUTO_TURN_MAX_POWER, Constants.Drive.AUTO_TURN_MAX_POWER)
        turnController.setAbsoluteTolerance(Constants.Drive.TURN_TOLERANCE_DEGREES)
        turnController.setContinuous(true)

        leftReceiver = PIDOutputReceiver()
        leftController = PIDController(Constants.Gains.FORWARD_P, Constants.Gains.FORWARD_I, Constants.Gains.FORWARD_D, Constants.Gains.FORWARD_F, leftEncoder, leftReceiver)
        leftController.setInputRange(-200.0, 200.0)
        leftController.setOutputRange(-Constants.Drive.AUTO_FORWARD_MAX_POWER, Constants.Drive.AUTO_FORWARD_MAX_POWER)
        leftController.setAbsoluteTolerance(Constants.Drive.FORWARD_TOLERANCE_INCHES)
        leftController.setContinuous(false)

        rightReceiver = PIDOutputReceiver()
        rightController = PIDController(Constants.Gains.FORWARD_P, Constants.Gains.FORWARD_I, Constants.Gains.FORWARD_D, Constants.Gains.FORWARD_F, rightEncoder, rightReceiver)
        rightController.setInputRange(-200.0, 200.0)
        rightController.setOutputRange(-Constants.Drive.AUTO_FORWARD_MAX_POWER, Constants.Drive.AUTO_FORWARD_MAX_POWER)
        rightController.setAbsoluteTolerance(Constants.Drive.FORWARD_TOLERANCE_INCHES)
        rightController.setContinuous(false)

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
        SmartDashboard.putNumber("leftEncoder", leftEncoder.distance)
        SmartDashboard.putNumber("rightEncoder", rightEncoder.distance)
        SmartDashboard.putNumber("leftEncoderRaw", leftEncoder.get().toDouble())
        SmartDashboard.putNumber("rightEncoderRaw", rightEncoder.get().toDouble())
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
        leftEncoder.stopLiveWindowMode()
        rightEncoder.stopLiveWindowMode()
        leftController.stopLiveWindowMode()
        rightController.stopLiveWindowMode()
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
        leftEncoder.reset()
        rightEncoder.reset()
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
        turnController.disable()
        leftController.disable()
        rightController.disable()

        setLeftRightPower(signal.leftMotor, signal.rightMotor)
    }

    @Synchronized
    fun setAutonomousDriving() {
        currentState = DriveControlState.AUTONOMOUS_DRIVING
    }

    fun setForwardSetpoint(distance: Double) {
        println("Set setpoint to $distance")
        setAutonomousDriving()
        leftController.enable()
        rightController.enable()
        leftController.setpoint = distance
        rightController.setpoint = distance
    }

    fun goForward(): Boolean {
        //        System.out.println("Moving rate of right: " + rightReceiver.getOutput() + " Distance: " + rightEncoder.pidGet() + " Time: " + Timer.getFPGATimestamp());
        //        System.out.println("Moving rate of left: " + leftReceiver.getOutput() + " Distance: " + leftEncoder.pidGet() + " Time: " + Timer.getFPGATimestamp());
        //        System.out.println("Left Setpoint: " + leftController.getSetpoint());
        //        System.out.println("Right Setpoint: " + rightController.getSetpoint());

        //        double leftError = Math.abs(leftController.getSetpoint())
        //        lastForwardErrors.add(Math.abs(left))
        if (leftController.onTarget() || currentState != DriveControlState.AUTONOMOUS_DRIVING) {
            return true
        }
        setLeftRightPower(-leftReceiver.output, -leftReceiver.output - rightReceiver.output * .15)
        return false
    }

    fun finishForward() {
        leftController.disable()
        rightController.disable()
        setOpenLoop(DriveSignal.NEUTRAL)
    }

    @Synchronized
    fun setAutonomousTurning() {
        currentState = DriveControlState.AUTONOMOUS_TURNING
    }

    fun setAngleSetpoint(angle: Double) {
        println("Set setpoint to " + angle)
        setAutonomousTurning()
        turnController.enable()
        turnController.setpoint = angle
    }

    fun turnAngle(): Boolean {
        println("Turn setpoint: " + turnController.setpoint + " Angle: " + ahrs.yaw + " Time: " + Timer.getFPGATimestamp())
        //        lastTurnErrors.add(Math.abs(turnController.getSetpoint() - ahrs.getYaw()));
        if (turnController.onTarget() || currentState != DriveControlState.AUTONOMOUS_TURNING) {
            return true
        }
        setLeftRightPower(-turnReceiver.output, turnReceiver.output)
        return false
    }

    fun finishTurn() {
        println("Finished turn")
        turnController.disable()
        setOpenLoop(DriveSignal.NEUTRAL)
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
