package org.usfirst.frc.team4099.robot

import edu.wpi.first.wpilibj.IterativeRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.auto.AutoModeExecuter
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.lib.util.CrashTracker
import org.usfirst.frc.team4099.lib.util.LatchedBoolean
import org.usfirst.frc.team4099.lib.util.ReflectingCSVWriter
import org.usfirst.frc.team4099.lib.util.SignalTable
import org.usfirst.frc.team4099.robot.drive.CheesyDriveHelper
import org.usfirst.frc.team4099.robot.drive.TankDriveHelper
import org.usfirst.frc.team4099.robot.loops.BrownoutDefender
import org.usfirst.frc.team4099.robot.loops.Looper
import org.usfirst.frc.team4099.robot.loops.VoltageEstimator
import org.usfirst.frc.team4099.robot.subsystems.Drive
import org.usfirst.frc.team4099.robot.subsystems.Elevator
import org.usfirst.frc.team4099.robot.subsystems.Intake
import org.usfirst.frc.team4099.robot.subsystems.Wrist

class Robot : IterativeRobot() {

    private val drive = Drive.instance

    private val cheesyDriveHelper = CheesyDriveHelper.instance
    private val tankDriveHelper = TankDriveHelper.instance
    private val intake = Intake.instance
    private val elevator = Elevator.instance
    private val wrist = Wrist.instance

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
            CrashTracker.logRobotInit()

//            DashboardConfigurator.initDashboard()

            //TODO: add the robot state estimator here
//            CameraServer.getInstance().startAutomaticCapture()
            enabledLooper.register(drive.loop)
//            enabledLooper.register(intake.loop)
//            enabledLooper.register(elevator.loop)
//            enabledLooper.register(wrist.loop)

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

//            val allianceOwnership = DashboardConfigurator.updateAllianceOwnership()

//            autoModeExecuter = AutoModeExecuter()
//            autoModeExecuter?.setAutoMode(DashboardConfigurator.getSelectedAutoMode(allianceOwnership))
//            autoModeExecuter?.start()

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
            val reverseIntake = controls.reverseIntake
            val openIntake = controls.openIntake
            val closeIntake = controls.closeIntake

            SmartDashboard.putBoolean("isQuickTurn", isQuickTurn)
            SmartDashboard.putNumber("voltage", VoltageEstimator.instance.averageVoltage)

//            if (controls.test) {
//                println("testing")
//                drive.setVelocitySetpoint(600 * throttle, 600 * throttle)
//            } else {
//                if (drive.highGear && shiftToLowGear) {
//                    drive.highGear = false
//                    println("Shifting to low gear")
//                } else if (!drive.highGear && shiftToHighGear) {
//                    drive.highGear = true
//                    println("Shifting to high gear")
//                }
//                if (intake.open && closeIntake) {
//                    intake.open = false
//                    println("Closing intake")
//                } else if (!intake.open && openIntake) {
//                    intake.open = true
//                    println("Opening intake")
//                }
            println(throttle)
                drive.setOpenLoop(cheesyDriveHelper.curvatureDrive(throttle, turn, isQuickTurn))
//            }

//            if (reverseIntake) {
//                intake.intakeState = Intake.IntakeState.OUT
//            } else if (intake.intakeState != Intake.IntakeState.SLOW) {
//                intake.intakeState = Intake.IntakeState.IN
//            }
//
            when {
                controls.elevatorTop -> elevator.elevatorState = Elevator.ElevatorState.HIGH
                controls.elevatorBottom -> elevator.elevatorState = Elevator.ElevatorState.LOW
                else ->  elevator.setOpenLoop(controls.elevatorPower)
            }

//            when {
//                controls.wristTop -> wrist.wristState = Wrist.WristState.STOWED_UP
//                controls.wristBottom -> wrist.wristState = Wrist.WristState.HORIZONTAL
//                else -> wrist.setOpenLoop(controls.wristPower)
//            }

            signalTable.sensorPosition = elevator.talon.sensorCollection.quadraturePosition
            signalTable.sensorVelocity = elevator.talon.sensorCollection.quadratureVelocity
            signalTable.closedLoopError = elevator.talon.getClosedLoopError(0)
            signalTable.activePosition = elevator.talon.activeTrajectoryPosition
            signalTable.activeVelocity = elevator.talon.activeTrajectoryVelocity
            signalTable.elevatorVoltage = elevator.talon.motorOutputVoltage
            signalTable.elevatorCurrent = elevator.talon.outputCurrent
            csvWriter.add(signalTable)

//            println(wrist.talon.sensorCollection.quadraturePosition)
            println("Current: ${elevator.talon.outputCurrent}")
            println("Voltage: ${elevator.talon.motorOutputVoltage}")
            println("Output: ${elevator.talon.motorOutputPercent}")

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

//    override fun testPeriodic() = teleopPeriodic()

    override fun testPeriodic() {
        val initial = Timer.getFPGATimestamp()
        while (Timer.getFPGATimestamp() - initial < 300) {
            drive.setOpenLoop(DriveSignal(1.0, 1.0))
        }
        drive.setOpenLoop(DriveSignal(0.0, 0.0))
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
