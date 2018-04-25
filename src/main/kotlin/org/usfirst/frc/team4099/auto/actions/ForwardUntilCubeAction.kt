package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.lib.util.Utils
import org.usfirst.frc.team4099.robot.subsystems.Drive
import org.usfirst.frc.team4099.robot.subsystems.Intake

class ForwardUntilCubeAction(private val timeout: Double) : Action {
    private val mDrive: Drive = Drive.instance
    private val intake = Intake.instance
    private var startDist: Double = 0.toDouble()
    private var otherStart: Double = 0.0
    private var power: Double = 0.toDouble()
    private var startAngle: Double = 0.toDouble()
    private var resetGyro: Boolean = false
    private var done: Boolean = false
    private var startTime = 0.0

    constructor(timeout: Double, resetGyro: Boolean) : this(timeout) {
        this.resetGyro = resetGyro
    }

    init {
        power = 0.5
    }

    override fun isFinished(): Boolean {
        return Timer.getFPGATimestamp() - timeout > 3 || intake.switchPressed
    }

    override fun update() {
        intake.intakeState = Intake.IntakeState.IN
        val ahrs = mDrive.getAHRS()
        val yaw = ahrs!!.yaw.toDouble()
        //        double correctionAngle = Math.IEEEremainder(yaw - startAngle, 360);
        val correctionAngle = startAngle - yaw
        if (Math.abs(correctionAngle) > 30) {
            done = true
            return
        }
        mDrive.arcadeDrive(power, correctionAngle * 0.01)
        //        System.out.println("yaw: " + yaw);
        println("correctionAngle: " + correctionAngle)
    }

    override fun done() {
        mDrive.setOpenLoop(DriveSignal.NEUTRAL)
        println("------- END FORWARD -------")
    }

    override fun start() {
        if (resetGyro) {
            while (!Utils.around(mDrive.getAHRS()!!.yaw.toDouble(), 0.0, 1.0)) {
                mDrive.getAHRS()!!.zeroYaw()
            }
            Timer.delay(1.0)
        }
        startTime = Timer.getFPGATimestamp()
        startAngle = mDrive.getAHRS()!!.yaw.toDouble()
        println("------- NEW START AUTONOMOUS RUN -------")
        println("Starting angle: " + startAngle)
        startDist = mDrive.getLeftDistanceInches()
        otherStart = mDrive.getRightDistanceInches()
    }
}