package org.usfirst.frc.team4099.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import org.usfirst.frc.team4099.DashboardConfigurator
import org.usfirst.frc.team4099.auto.AutoModeExecuter
import org.usfirst.frc.team4099.lib.util.CrashTracker
import org.usfirst.frc.team4099.lib.util.LatchedBoolean
import org.usfirst.frc.team4099.lib.util.ReflectingCSVWriter
import org.usfirst.frc.team4099.lib.util.SignalTable
import org.usfirst.frc.team4099.robot.drive.CheesyDriveHelper
import org.usfirst.frc.team4099.robot.drive.TankDriveHelper
import org.usfirst.frc.team4099.robot.loops.BrownoutDefender
import org.usfirst.frc.team4099.robot.loops.Looper
import org.usfirst.frc.team4099.robot.loops.VoltageEstimator
import org.usfirst.frc.team4099.robot.subsystems.*

class Robot : IterativeRobot() {

    private val drive = Drive.instance

    private val cheesyDriveHelper = CheesyDriveHelper.instance
    private val tankDriveHelper = TankDriveHelper.instance
    private val intake = Intake.instance
    private val elevator = Elevator.instance
    private val wrist = Wrist.instance
    private val forks = Forks.instance
    private val climber = Climber.instance

    private val controls = ControlBoard.instance
    private val disabledLooper = Looper("disabledLooper")
    private val enabledLooper = Looper("enabledLooper")

    private var autoModeExecuter: AutoModeExecuter? = null

    private val csvWriter = ReflectingCSVWriter<SignalTable>("/home/lvuser/out.csv", SignalTable::class.java)
    private val signalTable = SignalTable()

    private val logging = true
    private var isTurning = true
    private var isHighGear = LatchedBoolean()

    init {
        CrashTracker.logRobotConstruction()
    }


    override fun robotInit() {
        try {
//            CameraServer.getInstance().startAutomaticCapture()
            CrashTracker.logRobotInit()

            DashboardConfigurator.initDashboard()

            //TODO: add the robot state estimator here
            enabledLooper.register(drive.loop)
            enabledLooper.register(intake.loop)
            enabledLooper.register(elevator.loop)
            enabledLooper.register(wrist.loop)
            enabledLooper.register(forks.loop)
            enabledLooper.register(climber.loop)

            elevator.zeroSensors()

//            enabledLooper.register(CameraSwitcher.instance)
            enabledLooper.register(BrownoutDefender.instance)

            disabledLooper.register(VoltageEstimator.instance)
//            disabledLooper.register(CameraSwitcher.instance)


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

            if (csvWriter.linesToWrite.size > 0) {
                csvWriter.flush()
            }
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

            val allianceOwnership = DashboardConfigurator.updateAllianceOwnership()

            forks.latched = true
            autoModeExecuter = AutoModeExecuter()
            autoModeExecuter?.setAutoMode(DashboardConfigurator.getSelectedAutoMode(allianceOwnership))
            autoModeExecuter?.start()

        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("autonomousInit", t)
            throw t
        }

    }

    override fun teleopInit() {
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
            val shiftToHighGear = controls.switchToHighGear
            val shiftToLowGear = controls.switchToLowGear
            val reverseIntakeFast = controls.reverseIntakeFast
            val reverseIntakeSlow = controls.reverseIntakeSlow
            val openIntake = controls.openIntake
            val closeIntake = controls.closeIntake

//            SmartDashboard.putBoolean("isQuickTurn", isQuickTurn)
//            SmartDashboard.putNumber("voltage", VoltageEstimator.instance.averageVoltage)

            if (drive.highGear && shiftToLowGear) {
                drive.highGear = false
                println("Shifting to low gear")
            } else if (!drive.highGear && shiftToHighGear) {
                drive.highGear = true
                println("Shifting to high gear")
            }

            drive.setOpenLoop(cheesyDriveHelper.curvatureDrive(throttle, turn, isQuickTurn))

            if (intake.open && closeIntake) {
                intake.open = false
                println("Closing intake")
            } else if (!intake.open && openIntake) {
                intake.open = true
                println("Opening intake")
            }

            intake.intakeState = when {
                reverseIntakeFast -> Intake.IntakeState.FAST_OUT
                reverseIntakeSlow -> Intake.IntakeState.SLOW_OUT
                controls.runIntake -> Intake.IntakeState.IN
                intake.intakeState != Intake.IntakeState.SLOW -> Intake.IntakeState.STOP
                else -> intake.intakeState
            }

            if (controls.deployForks) {
                forks.latched = false
            }

//            elevator.s+etOpenLoop(controls.elevatorPower)

//            if (controls.test)
//            else {
//            SmartDashboard.putNumber("elevator/closedLoopTarget", target)
////            }
//            val wristTarget = controls.wristPower * 1000
//            wrist.setWristVelocity(wristTarget)

//            intake.intakeState = if (reverseIntake) Intake.IntakeState.OUT else Intake.IntakeState.IN
            // things such as is aligned?, etc

//            elevator.s+etOpenLoop(controls.elevatorPower)

//            if (controls.test)
//            else {
            val target = controls.elevatorPower * 1800
            elevator.setElevatorVelocity(target)
//            SmartDashboard.putNumber("elevator/closedLoopTarget", target)
////            }
<<<<<<< HEAD
//            val wristTarget = controls.wristPower * 1000
//            wrist.setWristVelocity(wristTarget)
            wrist.setOpenLoop(controls.wristPower)
=======
            if(controls.test) {
                wrist.setOpenLoop(controls.wristPower)
            } else {
                wrist.setWristVelocity(controls.wristPower * 650)
            }

>>>>>>> 4fbe075... Add mostly-working wrist PID

            climber.climberState = when {
                controls.runClimber -> Climber.ClimberState.CLIMBING
                controls.unClimber -> Climber.ClimberState.UNCLIMBING
                else -> Climber.ClimberState.NOT_CLIMBING
            }

//            intake.intakeState = if (reverseIntake) Intake.IntakeState.OUT else Intake.IntakeState.IN

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
            elevator.zeroSensors()
            wrist.zeroSensors()
            isTurning = true
            LiveWindow.setEnabled(true)
            startLiveWindowMode()
        } catch (t: Throwable) {
            CrashTracker.logThrowableCrash("testInit", t)
            throw t
        }

    }

//    override fun testPeriodic() {
//        try {
//            LiveWindow.run()
//            if (isTurning) {
//                //                isTurning = !mDrive.goForward()
//            } else {
//                drive.setOpenLoop(DriveSignal.NEUTRAL)
//            }
//            println("isTurning:" + isTurning)
//            //            DriveSignal lSignal = new DriveSignal(1, -1);
//            //            DriveSignal rSignal = new DriveSignal(-1, 1);
//
//
//            outputAllToSmartDashboard()
//            updateLiveWindowTables()
//            updateDashboardFeedback()
//        } catch (t: Throwable) {
//
//            CrashTracker.logThrowableCrash("testPeriodic", t)
//            throw t
//        }
//
//    }

    override fun testPeriodic() = teleopPeriodic()

    /**
     * Log information from all subsystems onto the SmartDashboard
     */
    private fun outputAllToSmartDashboard() {
        drive.outputToSmartDashboard()
        intake.outputToSmartDashboard()
        wrist.outputToSmartDashboard()
        elevator.outputToSmartDashboard()
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
