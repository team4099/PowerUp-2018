package org.usfirst.frc.team4099.robot.subsystems


import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.motorcontrol.ControlMode

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.loops.Loop

import org.usfirst.frc.team4099.robot.Kinematics
import org.usfirst.frc.team4099.robot.RobotState
import org.usfirst.frc.team4099.robot.loops.Looper
import org.usfirst.frc.team4099.lib.util.ReflectingCSVWriter
import org.usfirst.frc.team4099.lib.util.Utils
import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.control.PathFollower.Companion.Parameters
import org.usfirst.frc.team4099.lib.util.control.PathFollower
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Twist2D
import org.usfirst.frc.team4099.lib.util.drivers.NavX
import java.util.Arrays
import java.util.Optional
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import org.usfirst.frc.team4099.lib.util.control.Lookahead
import org.usfirst.frc.team4099.robot.Robot
import java.security.Policy

class Drive private constructor() : Subsystem {

    private val leftMasterSRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_MASTER_ID)
    private val leftSlave1SRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_SLAVE_1_ID)
    private val leftSlave2SRX: TalonSRX = TalonSRX(Constants.Drive.LEFT_SLAVE_2_ID)
    private val rightMasterSRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_MASTER_ID)
    private val rightSlave1SRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_SLAVE_1_ID)
    private val rightSlave2SRX: TalonSRX = TalonSRX(Constants.Drive.RIGHT_SLAVE_2_ID)
    private val ahrs: AHRS

    private var brakeMode: NeutralMode = NeutralMode.Coast //sets whether the break mode should be coast (no resistence) or by force

    private val pneumaticShifter: Solenoid = Solenoid(Constants.Drive.SHIFTER_MODULE,Constants.Drive.SHIFTER_CHANNEL)
    private var highGear: Boolean = false
    private var onTarget: Boolean = false
    private var isApproaching: Boolean = false

    private var driveControlState: DriveControlState = DriveControlState.VELOCITY_SETPOINT
    private val navXBoard: NavX

    private var robotState = RobotState.getInstance()
    private var pathFollower: PathFollower? = null

    private var targetHeading: Rotation2D = Rotation2D()
    private var currentPath: Path? = null

    private val csvWriter: ReflectingCSVWriter<PathFollower.Companion.DebugOutput>//see end of constructor to fix error

    enum class DriveControlState {
        OPEN_LOOP,
        VELOCITY_SETPOINT,
        PATH_FOLLOWING,
        TURN_TO_HEADING //turn in place
    }

    private var currentState = DriveControlState.OPEN_LOOP

    init {
        leftSlave1SRX.set(ControlMode.Follower, Constants.Drive.LEFT_MASTER_ID.toDouble()) //makes slaves
        leftSlave2SRX.set(ControlMode.Follower, Constants.Drive.LEFT_MASTER_ID.toDouble())
        leftMasterSRX.set(ControlMode.PercentOutput, 0.0) //changes Control Mode
        leftMasterSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0) //configs sensor to a quad encoder
        leftMasterSRX.setSensorPhase(false) //to align positive sensor velocity with positive motor output
        leftMasterSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 0)
        leftMasterSRX.config_kP(0, Constants.Gains.LEFT_LOW_KP, 0) //sets PIDF values
        leftMasterSRX.config_kI(0, Constants.Gains.LEFT_LOW_KI, 0)
        leftMasterSRX.config_kD(0, Constants.Gains.LEFT_LOW_KD, 0)
        leftMasterSRX.config_kF(0, Constants.Gains.LEFT_LOW_KF, 0)

        leftMasterSRX.config_kP(1, Constants.Gains.LEFT_HIGH_KP, 0)
        leftMasterSRX.config_kI(1, Constants.Gains.LEFT_HIGH_KI, 0)
        leftMasterSRX.config_kD(1, Constants.Gains.LEFT_HIGH_KD, 0)
        leftMasterSRX.config_kF(1, Constants.Gains.LEFT_HIGH_KF, 0)



        rightSlave1SRX.set(ControlMode.Follower, Constants.Drive.RIGHT_MASTER_ID.toDouble()) //makes slaves
        rightSlave2SRX.set(ControlMode.Follower, Constants.Drive.RIGHT_MASTER_ID.toDouble())
        rightMasterSRX.set(ControlMode.PercentOutput, 0.0) //changes Control Mode
        rightMasterSRX.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0) //configs sensor to a quad encoder
        rightMasterSRX.setSensorPhase(false) //to align positive sensor velocity with positive motor output
        rightMasterSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 0)
        rightMasterSRX.config_kP(0, Constants.Gains.RIGHT_LOW_KP, 0) //sets PIdF values
        rightMasterSRX.config_kI(0, Constants.Gains.RIGHT_LOW_KI, 0)
        rightMasterSRX.config_kD(0, Constants.Gains.RIGHT_LOW_KD, 0)
        rightMasterSRX.config_kF(0, Constants.Gains.RIGHT_LOW_KF, 0)

        rightMasterSRX.config_kP(1, Constants.Gains.RIGHT_HIGH_KP, 0)
        rightMasterSRX.config_kI(1, Constants.Gains.RIGHT_HIGH_KI, 0)
        rightMasterSRX.config_kD(1, Constants.Gains.RIGHT_HIGH_KD, 0)
        rightMasterSRX.config_kF(1, Constants.Gains.RIGHT_HIGH_KF, 0)

        leftMasterSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0)
        leftMasterSRX.configVelocityMeasurementWindow(32, 0)
        rightMasterSRX.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0)
        rightMasterSRX.configVelocityMeasurementWindow(32, 10)

        setOpenLoop(DriveSignal.NEUTRAL)

        ahrs = AHRS(SPI.Port.kMXP)

        this.zeroSensors()

        //reloadGains()
        highGear = false
        setHighGear(true)
        setOpenLoop(DriveSignal.NEUTRAL)
        navXBoard = NavX(SPI.Port.kMXP)
        brakeMode = NeutralMode.Brake
        setBrakeMode(NeutralMode.Coast)
        csvWriter = ReflectingCSVWriter<PathFollower.Companion.DebugOutput>(
                "file.csv", PathFollower.Companion.DebugOutput::class.java) //todo: add debug file
    }

    fun registerEnabledLoops(inloop: Looper) {
        inloop.register(loop)
    }

    @Synchronized
    fun setOpenLoop(signal: DriveSignal) {
        if (currentState !== DriveControlState.OPEN_LOOP) {
            leftMasterSRX.set(ControlMode.PercentOutput, 0.0)
            rightMasterSRX.set(ControlMode.PercentOutput, 0.0)
            leftMasterSRX.configNominalOutputForward(0.0, 0)
            rightMasterSRX.configNominalOutputForward(0.0, 0)
            currentState = DriveControlState.OPEN_LOOP
            setBrakeMode(NeutralMode.Coast)
        }
        setLeftRightPower(signal.leftMotor, signal.rightMotor)
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


    fun getHighGear(): Boolean {
        return highGear
    }


    fun getBrakeMode(): NeutralMode {
        return brakeMode
    }

    fun setBrakeMode(type: NeutralMode) {
        if (brakeMode != type) {
            brakeMode = type
            rightMasterSRX.setNeutralMode(type)
            rightSlave1SRX.setNeutralMode(type)
            rightSlave2SRX.setNeutralMode(type)
            leftMasterSRX.setNeutralMode(type)
            leftSlave1SRX.setNeutralMode(type)
            leftSlave2SRX.setNeutralMode(type)
        }
    }

    override fun stop() {
        synchronized(this) {
            setOpenLoop(DriveSignal.NEUTRAL)
        }
    }

    @Synchronized
    fun resetEncoders() {
        leftMasterSRX.setSelectedSensorPosition(0, 0, 0)
        leftMasterSRX.sensorCollection.setPulseWidthPosition(0, 0)
        leftSlave1SRX.setSelectedSensorPosition(0, 0, 0)
        leftSlave2SRX.setSelectedSensorPosition(0, 0, 0)
        rightMasterSRX.setSelectedSensorPosition(0, 0, 0)
        rightMasterSRX.sensorCollection.setPulseWidthPosition(0, 0)
        rightSlave1SRX.setSelectedSensorPosition(0, 0, 0)
        rightSlave2SRX.setSelectedSensorPosition(0, 0, 0)

    }

    fun getAHRS(): AHRS? {
        return if (ahrs.isConnected) ahrs else null
    }

    override fun outputToSmartDashboard() {
        if (this.getAHRS() != null) {
            SmartDashboard.putNumber("gyro", this.getAHRS()!!.yaw.toDouble())
        } else {
            SmartDashboard.putNumber("gyro", -31337.0) //todo: add more data if you want
        }
        SmartDashboard.putNumber("leftTalon", leftMasterSRX.motorOutputVoltage)
        SmartDashboard.putNumber("rightTalon", rightMasterSRX.motorOutputVoltage)
    }

    fun startLiveWindowMode() {
        LiveWindow.addSensor("Drive", "Gyro", ahrs)
    }

    fun stopLiveWindowMode() {
        //TODO
    }

    fun updateLiveWindowTables() {

    }

    override fun zeroSensors() {
        if (ahrs.isConnected) {
            ahrs.reset()
        }
        resetEncoders()
        navXBoard.zeroYaw()
    }

    @Synchronized
    fun setVelocitySetpoint(leftInchesPerSec: Double, rightInchesPerSec: Double) {
        configureTalonsForVelocityControl()
        currentState = DriveControlState.VELOCITY_SETPOINT
        updateVelocitySetpoint(leftInchesPerSec, rightInchesPerSec)
    }

    @Synchronized
    private fun updateVelocitySetpoint(leftInchesPerSec: Double, rightInchesPerSec: Double) {
        if (usesTalonVelocityControl(currentState)) {
            val maxDesired: Double = Math.max(Math.abs(leftInchesPerSec), Math.abs(rightInchesPerSec))
            var scale: Double
            if (maxDesired > Constants.Drive.HIGH_GEAR_MAX_SETPOINT) {
                scale = Constants.Drive.HIGH_GEAR_MAX_SETPOINT / maxDesired
            } else {
                scale = 1.0
            }
            leftMasterSRX.set(ControlMode.Velocity, inchesPerSecondToRpm(leftInchesPerSec * scale))
            rightMasterSRX.set(ControlMode.Velocity, inchesPerSecondToRpm(rightInchesPerSec * scale))
        } else {
            print("Incorrect Velocity Control Mode")
            leftMasterSRX.set(ControlMode.Velocity, 0.0)
            rightMasterSRX.set(ControlMode.Velocity, 0.0)
        }

    }

    @Synchronized
    private fun updatePositionSetpoint(leftPositionInches: Double, rightPositionInches: Double) {
        if (usesTalonPositionControl(currentState)) {
            leftMasterSRX.set(ControlMode.MotionMagic, leftPositionInches)
            leftMasterSRX.set(ControlMode.MotionMagic, leftPositionInches)
        } else {
            println("Bad position control state")
            leftMasterSRX.set(ControlMode.MotionMagic, 0.0)
            rightMasterSRX.set(ControlMode.MotionMagic, 0.0)
        }
    }

    private fun configureTalonsForVelocityControl() { //should further review cause im bad
        if (!usesTalonVelocityControl(currentState)) {
            // We entered a velocity control state.
            leftMasterSRX.set(ControlMode.Velocity, 0.0) //velocity  output value is in position change / 100ms
            leftMasterSRX.configNominalOutputForward(Constants.Velocity.DRIVE_HIGH_GEAR_NOMINAL_OUTPUT, 0)
            leftMasterSRX.configNominalOutputReverse(Constants.Velocity.DRIVE_HIGH_GEAR_NOMINAL_OUTPUT, 0)
            leftMasterSRX.selectProfileSlot(Constants.Velocity.HIGH_GEAR_VELOCITY_CONTROL_SLOT, 0)
            leftMasterSRX.configPeakOutputForward(Constants.Velocity.DRIVE_HIGH_GEAR_MAX_FORWARD_OUTPUT, 0)
            leftMasterSRX.configPeakOutputReverse(Constants.Velocity.DRIVE_HIGH_GEAR_MAX_REVERSE_OUTPUT, 0)
            rightMasterSRX.set(ControlMode.Velocity, 0.0) //velocity  output value is in position change / 100ms
            rightMasterSRX.configNominalOutputForward(Constants.Velocity.DRIVE_HIGH_GEAR_NOMINAL_OUTPUT, 0)
            rightMasterSRX.configNominalOutputReverse(Constants.Velocity.DRIVE_HIGH_GEAR_NOMINAL_OUTPUT, 0)
            rightMasterSRX.selectProfileSlot(Constants.Velocity.HIGH_GEAR_VELOCITY_CONTROL_SLOT, 0)
            rightMasterSRX.configPeakOutputForward(Constants.Velocity.DRIVE_HIGH_GEAR_MAX_FORWARD_OUTPUT, 0)
            rightMasterSRX.configPeakOutputReverse(Constants.Velocity.DRIVE_HIGH_GEAR_MAX_REVERSE_OUTPUT, 0)
            setBrakeMode(NeutralMode.Brake)
        }
    }

    private fun configureTalonsforPositionControl() {
        if (!usesTalonPositionControl(currentState)) {
            // We entered a position control state.
            leftMasterSRX.set(ControlMode.MotionMagic, 0.0)
            leftMasterSRX.configNominalOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            leftMasterSRX.configNominalOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            leftMasterSRX.selectProfileSlot(Constants.Velocity.LOW_GEAR_VELOCITY_CONTROL_SLOT, 0)
            leftMasterSRX.configPeakOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_MAX_FORWARD_OUTPUT, 0)
            leftMasterSRX.configPeakOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_MAX_REVERSE_OUTPUT, 0)
            rightMasterSRX.set(ControlMode.MotionMagic, 0.0)
            rightMasterSRX.configNominalOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            rightMasterSRX.configNominalOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_NOMINAL_OUTPUT, 0)
            rightMasterSRX.selectProfileSlot(Constants.Velocity.LOW_GEAR_VELOCITY_CONTROL_SLOT, 0)
            rightMasterSRX.configPeakOutputForward(Constants.Velocity.DRIVE_LOW_GEAR_MAX_FORWARD_OUTPUT, 0)
            rightMasterSRX.configPeakOutputReverse(Constants.Velocity.DRIVE_LOW_GEAR_MAX_REVERSE_OUTPUT, 0)
            setBrakeMode(NeutralMode.Brake)
        }
    }


    fun isHighGear() : Boolean {
        return highGear
    }

    fun setHighGear(wantsHighGear: Boolean) {
        if (wantsHighGear != highGear) {
            highGear = wantsHighGear
            pneumaticShifter.set(!wantsHighGear)
        }
    }

    /**
     * Powers the left and right talons during OPEN_LOOP
     * @param left
     * @param right
     */
    @Synchronized
    fun setLeftRightPower(left: Double, right: Double) {
        leftMasterSRX.set(ControlMode.PercentOutput, -left)
        rightMasterSRX.set(ControlMode.PercentOutput, right)
    }


    val loop: Loop = object : Loop {
        override fun onStart(timestamp: Double) {
            setOpenLoop(DriveSignal.NEUTRAL)
            setBrakeMode(NeutralMode.Coast)
            setVelocitySetpoint(0.0,0.0)
            navXBoard.reset()
        }

        override fun onLoop(timestamp: Double) {
            synchronized(this@Drive) {
                when (currentState) {
                    DriveControlState.OPEN_LOOP -> {
                        return
                    }
                    DriveControlState.VELOCITY_SETPOINT -> {
                        return
                    }
                DriveControlState.PATH_FOLLOWING ->{
                    if (pathFollower != null) {
                        updatePathFollower(timestamp)
                        csvWriter.add(pathFollower!!.getDebug())
                    }
                }
                    DriveControlState.TURN_TO_HEADING -> {
                        updateTurnToHeading(timestamp)
                        return
                    }
                    else -> {
                        System.out.println("Unexpected drive control state: " + currentState)
                    }
                }
            }
        }

        override fun onStop(timestamp: Double) {
            stop()
            csvWriter.flush()
        }
    }

    /* private fun updateTurnToHeading(timestamp: Double) {
         /*if (Superstructure.getInstance().isShooting()) {
             // Do not update heading while shooting - just base lock. By not updating the setpoint, we will fight to
             // keep position.
             return
         }*/
         val field_to_robot = currentState.getLatestFieldToVehicle().getValue().getRotation()

         // Figure out the rotation necessary to turn to face the goal.
         val robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading)

         // Check if we are on target
         val kGoalPosTolerance = 0.75 // degrees
         val kGoalVelTolerance = 5.0 // inches per second
         if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
                 && Math.abs(getLeftVelocityInchesPerSec()) < kGoalVelTolerance
                 && Math.abs(getRightVelocityInchesPerSec()) < kGoalVelTolerance) {
             // Use the current setpoint and base lock.
             mIsOnTarget = true
             updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches())
             return
         }

         val wheel_delta = Kinematics
                 .inverseKinematics(Twist2d(0, 0, robot_to_target.getRadians()))
         updatePositionSetpoint(wheel_delta.left + getLeftDistanceInches(),
                 wheel_delta.right + getRightDistanceInches())
     }*/

    private fun rotationsToInches(rotations: Double): Double {
        return rotations * (Constants.Wheels.DRIVE_WHEEL_DIAMETER_INCHES * Math.PI)
    }

    private fun rpmToInchesPerSecond(rpm: Double): Double {
        return rotationsToInches(rpm) / 60
    }

    private fun inchesToRotations(inches: Double): Double {
        return inches / (Constants.Wheels.DRIVE_WHEEL_DIAMETER_INCHES * Math.PI)
    }

    private fun inchesPerSecondToRpm(inches_per_second: Double): Double {
        return inchesToRotations(inches_per_second) * 60
    }

    fun getLeftDistanceInches(): Double {
        return rotationsToInches(leftMasterSRX.getSelectedSensorPosition(0).toDouble())
    }

    fun getRightDistanceInches(): Double {
        return rotationsToInches(rightMasterSRX.getSelectedSensorPosition(0).toDouble())
    }

    fun getLeftVelocityInchesPerSec(): Double {
        return rpmToInchesPerSecond(leftMasterSRX.getSelectedSensorVelocity(0).toDouble())
    }

    fun getRightVelocityInchesPerSec(): Double {
        return rpmToInchesPerSecond(rightMasterSRX.getSelectedSensorVelocity(0).toDouble())
    }

    @Synchronized fun getGyroAngle(): Rotation2D {
        return navXBoard.getYaw()
    }

    @Synchronized fun getNavXBoard(): NavX {
        return navXBoard
    }

    @Synchronized fun getGyroVelocityDegPerSec(): Double {
        return navXBoard.getYawRateDegPerSec()
    }

    fun updateTurnToHeading(timestamp: Double) {
        val fieldToRobot: Rotation2D = robotState.getLatestFieldToVehicle().value.getRotation()
        val robotToTarget: Rotation2D = fieldToRobot.inverse().rotateBy(targetHeading)

        val GOAL_POS_TOLER = 0.75
        val GOAL_VEL_TOLER = 5.0
        if (Math.abs(robotToTarget.degrees) < GOAL_POS_TOLER && Math.abs(getLeftVelocityInchesPerSec()) < GOAL_VEL_TOLER && Math.abs(getRightVelocityInchesPerSec()) < GOAL_VEL_TOLER) {
            onTarget = true
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches())
            return
        }

        val wheelDelta: Kinematics.DriveVelocity = Kinematics.inverseKinematics(Twist2D(0.0,0.0,robotToTarget.radians))
        updatePositionSetpoint(wheelDelta.left + getLeftDistanceInches(), wheelDelta.right + getRightDistanceInches())
    }

    fun updatePathFollower(timestamp: Double) {
        val robotPose: RigidTransform2D = robotState.getLatestFieldToVehicle().value
        var command: Twist2D = pathFollower!!.update(timestamp, robotPose, RobotState.getInstance().distDriven, RobotState.getInstance().predictedVehicleVel.dx())
        if (!pathFollower!!.isFinished()) {
            var setpoint: Kinematics.DriveVelocity = Kinematics.inverseKinematics(command)
            updateVelocitySetpoint(setpoint.left, setpoint.right)
        } else {
            updateVelocitySetpoint(0.0,0.0)
        }
    }

    @Synchronized fun isOnTarget(): Boolean {
        return onTarget
    }

    @Synchronized fun setWantTurnToHeading(heading: Rotation2D) {
        if (driveControlState != DriveControlState.TURN_TO_HEADING) {
            configureTalonsforPositionControl()
            driveControlState = DriveControlState.TURN_TO_HEADING
            updatePositionSetpoint(getLeftDistanceInches(), getRightDistanceInches())
        }
        if (Math.abs(heading.inverse().rotateBy(targetHeading).degrees) > 1E-3) {
            targetHeading = heading
            onTarget = false
        }
        highGear = false
    }

    @Synchronized fun setWantDrivePath(path: Path, reversed: Boolean) {
        if (currentPath != path || driveControlState != DriveControlState.PATH_FOLLOWING) {
            configureTalonsForVelocityControl()
            RobotState.getInstance().resetDistDriven()
            pathFollower = PathFollower(path, reversed, PathFollower.Companion.Parameters(Lookahead(Constants.PathFollowing.MIN_LOOK_AHEAD, Constants.PathFollowing.MAX_LOOK_AHEAD, Constants.PathFollowing.MIN_LOOK_AHEAD_SPEED, Constants.PathFollowing.MAX_LOOK_AHEAD_SPEED), Constants.PathFollowing.INERTIA_STEERING_GAIN, Constants.PathFollowing.PF_Kp, Constants.PathFollowing.PF_Ki, Constants.PathFollowing.PF_Kv, Constants.PathFollowing.PF_Kffv, Constants.PathFollowing.PF_Kffa, Constants.PathFollowing.PATH_FOLLOWING_MAX_VEL, Constants.PathFollowing.PATH_FOLLOWING_MAX_ACCEL, Constants.PathFollowing.PATH_FOLLOWING_GOAL_POS_TOL, Constants.PathFollowing.PATH_FOLLOWING_GOAL_VEL_TOL, Constants.PathFollowing.PATH_STOP_STEERING_DIST))
            driveControlState = DriveControlState.PATH_FOLLOWING
            currentPath = path
        } else {
            setVelocitySetpoint(0.0,0.0)
        }
    }

    @Synchronized fun isDoneWithPath(): Boolean {
        return if(driveControlState == DriveControlState.PATH_FOLLOWING && pathFollower != null) {
            pathFollower!!.isFinished()
        } else {
            System.out.println("Not Path Following Mode")
            true
        }
    }

    fun isApproaching(): Boolean {
        return isApproaching
    }

    @Synchronized fun isDoneWithTurn(): Boolean {
        return if (driveControlState == DriveControlState.TURN_TO_HEADING) {
            onTarget
        } else {
            System.out.println("Not turn to heading mode")
            false
        }
    }

    @Synchronized fun hasPassedMarker(marker: String): Boolean {
        return if (driveControlState == DriveControlState.PATH_FOLLOWING && pathFollower != null) {
            pathFollower!!.hasPassedMarker(marker)
        } else {
            System.out.println("Robot is not in path following mode")
            false
        }
    }

    @Synchronized fun getAccelX(): Double {
        return navXBoard.getRawAccelX().toDouble()
    }

    //fun checkSystem(): Boolean {}



    companion object {
        val instance = Drive()

        private val lowGearPositionControlSlot = 0
        private val highGearVelocityControlSlot = 1

        protected fun usesTalonVelocityControl(state: DriveControlState): Boolean {
            return state == DriveControlState.VELOCITY_SETPOINT || state == DriveControlState.PATH_FOLLOWING
        }

        protected fun usesTalonPositionControl(state: DriveControlState): Boolean {
            return false
        }
    }

}

