package org.usfirst.frc.team4099.robot

import edu.wpi.first.wpilibj.CameraServer
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.SmartDashboardInteractions
import org.usfirst.frc.team4099.auto.AutoModeExecuter
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.lib.util.CrashTracker
import org.usfirst.frc.team4099.robot.drive.CDriveHelper
import org.usfirst.frc.team4099.robot.drive.TankDriveHelper
import org.usfirst.frc.team4099.robot.loops.BrownoutDefender
import org.usfirst.frc.team4099.robot.loops.Looper
import org.usfirst.frc.team4099.robot.loops.VoltageEstimator
import org.usfirst.frc.team4099.robot.subsystems.Climber
import org.usfirst.frc.team4099.robot.subsystems.Drive
import org.usfirst.frc.team4099.robot.subsystems.Intake

class Robot : IterativeRobot() {

    private val mDrive = Drive.instance
    private val mIntake = Intake.instance
    private val mClimber = Climber.instance

    private val mCDriveHelper = CDriveHelper.instance
    private val mTDriveHelper = TankDriveHelper.instance

    private val mControls = ControlBoard.instance
    private val mDisabledLooper = Looper("disabledLooper")
    private val mEnabledLooper = Looper("enabledLooper")

    private var mAutoModeExecuter: AutoModeExecuter? = null
    private val mSmartDashboardInteractions = SmartDashboardInteractions()

    private val logging = true
    private var isTurning = true

    private var lastToggleIntakeClosed = false

    private val slowFactor = 1.0

    init {
        CrashTracker.logRobotConstruction()
    }


    override fun robotInit() {
        try {
            CrashTracker.logRobotInit()

            //TODO: add the robot state estimator here
            CameraServer.getInstance().startAutomaticCapture()
            mEnabledLooper.register(mDrive.loop)
            mEnabledLooper.register(mIntake.loop)
            mEnabledLooper.register(mClimber.loop)
            mEnabledLooper.register(BrownoutDefender.instance)

            mDisabledLooper.register(VoltageEstimator.instance)

            mSmartDashboardInteractions.initWithDefaults()

        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("robotInit", t)
            throw t
        }
    }

    override fun disabledInit() {
        try {
            CrashTracker.logDisabledInit()

            mEnabledLooper.stop() // end EnabledLooper
            mDisabledLooper.start() // start DisabledLooper

        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("disabledInit", t)
            throw t
        }

    }

    override fun autonomousInit() {
        try {
            CrashTracker.logAutoInit()

            if (mAutoModeExecuter != null) {
                mAutoModeExecuter!!.stop()
            }
            mAutoModeExecuter = null

            mDisabledLooper.stop() // end DisabledLooper
            mEnabledLooper.start() // start EnabledLooper
            mDrive.zeroSensors()
            mDrive.getAHRS()!!.zeroYaw()

            mAutoModeExecuter = AutoModeExecuter()
            mAutoModeExecuter!!.setAutoMode(mSmartDashboardInteractions.selectedAutonMode)
            mAutoModeExecuter!!.start()
            mSmartDashboardInteractions.isInHoodTuningMode

        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("autonomousInit", t)
            throw t
        }

    }

    override fun teleopInit() {
        try {
            CrashTracker.logTeleopInit()
            isTurning = true
            mEnabledLooper.start() // start EnabledLooper
            mDisabledLooper.stop() // end DisabledLooper

        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("teleopInit", t)
            throw t
        }

    }

    override fun disabledPeriodic() {
        try {
            outputAllToSmartDashboard()

        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("disabledPeriodic", t)
            throw t
        }

    }

    override fun autonomousPeriodic() {
        try {
            //            mDrive.turnAngle();
            outputAllToSmartDashboard()
            updateDashboardFeedback()
        } catch (t: Throwable) {

            CrashTracker.logThrowableCrash("autonomousPeriodic", t)
            throw t
        }

    }

    override fun teleopPeriodic() {
        try {
            val throttle = mControls.throttle
            val turn = mControls.turn
            val isQuickTurn = mControls.quickTurn

            val toggleIntake = mControls.toggleIntake
            val toggleIntakeClosed = mControls.toggleIntakeClosed
            val setIntakeUp = mControls.intakeUp
            val setIntakeDown = mControls.intakeDown
            //            boolean toggleSlowMode = mControls.getToggleSlowMode();

            val climbing = mControls.climber

            SmartDashboard.putBoolean("isQuickTurn", isQuickTurn)
            SmartDashboard.putNumber("voltage", VoltageEstimator.instance.averageVoltage)

            //            if (toggleSlowMode)
            //                slowFactor = 1.5 - slowFactor;

            mDrive.setOpenLoop(mCDriveHelper.curvatureDrive(throttle, turn, isQuickTurn))

            mIntake.updateIntake(toggleIntake)
            if (toggleIntakeClosed && !lastToggleIntakeClosed) {
                if (mIntake.intakePosition == Intake.IntakePosition.DOWN_AND_CLOSED)
                    mIntake.updateIntake(Intake.IntakePosition.DOWN_AND_OPEN)
                else if (mIntake.intakePosition == Intake.IntakePosition.DOWN_AND_OPEN)
                    mIntake.updateIntake(Intake.IntakePosition.DOWN_AND_CLOSED)
                else if (mIntake.intakePosition == Intake.IntakePosition.UP_AND_CLOSED)
                    mIntake.updateIntake(Intake.IntakePosition.UP_AND_OPEN)
                else if (mIntake.intakePosition == Intake.IntakePosition.UP_AND_OPEN)
                    mIntake.updateIntake(Intake.IntakePosition.UP_AND_CLOSED)
            } else if (setIntakeUp) {
                if (mIntake.intakePosition == Intake.IntakePosition.DOWN_AND_OPEN || mIntake.intakePosition == Intake.IntakePosition.UP_AND_OPEN)
                    mIntake.updateIntake(Intake.IntakePosition.UP_AND_OPEN)
                else
                    mIntake.updateIntake(Intake.IntakePosition.UP_AND_CLOSED)
            } else if (setIntakeDown) {
                if (mIntake.intakePosition == Intake.IntakePosition.DOWN_AND_OPEN || mIntake.intakePosition == Intake.IntakePosition.UP_AND_OPEN)
                    mIntake.updateIntake(Intake.IntakePosition.DOWN_AND_OPEN)
                else
                    mIntake.updateIntake(Intake.IntakePosition.DOWN_AND_CLOSED)
            }

            lastToggleIntakeClosed = toggleIntakeClosed

            if (climbing) {
                mClimber.setClimbingMode(Climber.ClimberState.CLIMBING)
            } else {
                mClimber.setClimbingMode(Climber.ClimberState.NOT_CLIMBING)
            }

            outputAllToSmartDashboard()
            updateDashboardFeedback() // things such as is aligned?, etc

        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("teleopPeriodic", t)
            throw t
        }

    }

    override fun testInit() {
        try {
            CrashTracker.logAutoInit()
            mEnabledLooper.start() // start EnabledLooper
            mDisabledLooper.stop() // end DisabledLooper
            mDrive.zeroSensors()
            isTurning = true
            LiveWindow.setEnabled(true)
            startLiveWindowMode()
        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("testInit", t)
            throw t
        }

    }

    override fun testPeriodic() {
        try {
            LiveWindow.run()
            if (isTurning) {
//                isTurning = !mDrive.goForward()
            } else {
                mDrive.setOpenLoop(DriveSignal.NEUTRAL)
            }
            println("isTurning:" + isTurning)
            //            DriveSignal lSignal = new DriveSignal(1, -1);
            //            DriveSignal rSignal = new DriveSignal(-1, 1);
            //            if(mControls.getClimber()) {
            //                mDrive.setOpenLoop(lSignal);
            //            } else {
            //                mDrive.setOpenLoop(rSignal);
            //            }

            outputAllToSmartDashboard()
            updateLiveWindowTables()
            updateDashboardFeedback()
        } catch (t: Throwable) {

            CrashTracker.logThrowableCrash("testPeriodic", t)
            throw t
        }

    }

    /**
     * Log information from all subsystems onto the SmartDashboard
     */
    private fun outputAllToSmartDashboard() {
        if (logging) {
            mDrive.outputToSmartDashboard() // subsystems output to SmartDashboard
            mIntake.outputToSmartDashboard()
            mClimber.outputToSmartDashboard()
        }
    }

    private fun startLiveWindowMode() {
        mDrive.startLiveWindowMode()
    }

    private fun updateLiveWindowTables() {
        mDrive.updateLiveWindowTables()
    }

    private fun updateDashboardFeedback() {
        // update things such as "is robot aligned with peg"
    }
}
