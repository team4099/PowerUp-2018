package org.usfirst.frc.team4099.robot

import edu.wpi.first.wpilibj.CameraServer
import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.AutonomousSelector
import org.usfirst.frc.team4099.auto.AutoModeExecuter
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.lib.util.CrashTracker
import org.usfirst.frc.team4099.lib.util.LatchedBoolean
import org.usfirst.frc.team4099.robot.drive.CheesyDriveHelper
import org.usfirst.frc.team4099.robot.drive.TankDriveHelper
import org.usfirst.frc.team4099.robot.loops.BrownoutDefender
import org.usfirst.frc.team4099.robot.loops.Looper
import org.usfirst.frc.team4099.robot.loops.VoltageEstimator
import org.usfirst.frc.team4099.robot.subsystems.Drive

class Robot : IterativeRobot() {

    private val drive = Drive.instance

    private val cheesyDriveHelper = CheesyDriveHelper.instance
    private val tankDriveHelper = TankDriveHelper.instance

    private val controls = ControlBoard.instance
    private val disabledLooper = Looper("disabledLooper")
    private val enabledLooper = Looper("enabledLooper")

    private var autoModeExecuter: AutoModeExecuter? = null

    private val logging = true
    private var isTurning = true
    private var isHighGear = LatchedBoolean()

    init {
        CrashTracker.logRobotConstruction()
    }


    override fun robotInit() {
        try {
            CrashTracker.logRobotInit()

            //TODO: add the robot state estimator here
            CameraServer.getInstance().startAutomaticCapture()
            enabledLooper.register(drive.loop)

            enabledLooper.register(BrownoutDefender.instance)

            disabledLooper.register(VoltageEstimator.instance)


        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("robotInit", t)
            throw t
        }
    }

    override fun disabledInit() {
        try {
            CrashTracker.logDisabledInit()

            enabledLooper.stop() // end EnabledLooper
            disabledLooper.start() // start DisabledLooper

        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("disabledInit", t)
            throw t
        }

    }

    override fun autonomousInit() {
        try {
            CrashTracker.logAutoInit()

            if (autoModeExecuter != null) {
                autoModeExecuter!!.stop()
            }
            autoModeExecuter = null

            disabledLooper.stop() // end DisabledLooper
            enabledLooper.start() // start EnabledLooper
            drive.zeroSensors()
            drive.getAHRS()?.zeroYaw()

            autoModeExecuter = AutoModeExecuter()
            autoModeExecuter?.setAutoMode(AutonomousSelector.getSelectedAutoMode())
            autoModeExecuter?.start()

        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("autonomousInit", t)
            throw t
        }

    }

    override fun teleopInit() {
        println("hello, in teleop")
        try {
            CrashTracker.logTeleopInit()
            isTurning = true
            enabledLooper.start() // start EnabledLooper
            disabledLooper.stop() // end DisabledLooper

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
            outputAllToSmartDashboard()
            updateDashboardFeedback()
        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("autonomousPeriodic", t)
            throw t
        }

    }

    override fun teleopPeriodic() {
        try {
            val throttle = controls.throttle
            val turn = controls.turn
            val isQuickTurn = controls.quickTurn


            println("teleop periodic")
            SmartDashboard.putBoolean("isQuickTurn", isQuickTurn)
            SmartDashboard.putNumber("voltage", VoltageEstimator.instance.averageVoltage)

            drive.setOpenLoop(cheesyDriveHelper.curvatureDrive(throttle, turn, isQuickTurn))

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
            enabledLooper.start() // start EnabledLooper
            disabledLooper.stop() // end DisabledLooper
            drive.zeroSensors()
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
                drive.setOpenLoop(DriveSignal.NEUTRAL)
            }
            println("isTurning:" + isTurning)
            //            DriveSignal lSignal = new DriveSignal(1, -1);
            //            DriveSignal rSignal = new DriveSignal(-1, 1);


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
            drive.outputToSmartDashboard() // subsystems output to SmartDashboard

        }
    }

    private fun startLiveWindowMode() {
        drive.startLiveWindowMode()
    }

    private fun updateLiveWindowTables() {
        drive.updateLiveWindowTables()
    }

    private fun updateDashboardFeedback() {
        // update things such as "is robot aligned with peg"
    }
}
