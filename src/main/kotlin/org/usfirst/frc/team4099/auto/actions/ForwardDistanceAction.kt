package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.drive.DriveSignal
import org.usfirst.frc.team4099.lib.util.Utils
import org.usfirst.frc.team4099.robot.subsystems.Drive

/**
 * Created by Oksana on 2/16/2017.
 */
class ForwardDistanceAction(initInchesToMove: Double) : Action {
    private val mDrive: Drive = Drive.instance
    private val inchesToMove: Double = Math.abs(initInchesToMove)
    private var startDist: Double = 0.toDouble()
    private var otherStart: Double = 0.0
    private val direction: Int
    private var power: Double = 0.toDouble()
    private var startAngle: Double = 0.toDouble()
    private var resetGyro: Boolean = false
    private var done: Boolean = false
    private var startTime = 0.0

    constructor(inchesToMove: Double, slowMode: Boolean, resetGyro: Boolean) : this(inchesToMove) {
        if (slowMode) {
            this.power = .6
        }
        this.resetGyro = resetGyro
    }

    init {
        direction = inchesToMove.toInt() / initInchesToMove.toInt()
        this.power = 1.0
    }

    override fun isFinished(): Boolean {
        return Math.abs(mDrive.getLeftDistanceInches()) - startDist >= inchesToMove || Math.abs(mDrive.getRightDistanceInches()) - otherStart >= inchesToMove || done || Timer.getFPGATimestamp() - startTime > 3
    }

    override fun update() {
        val ahrs = mDrive.getAHRS()
        val yaw = ahrs!!.yaw.toDouble()
        //        double correctionAngle = Math.IEEEremainder(yaw - startAngle, 360);
        val correctionAngle = startAngle - yaw
        if (Math.abs(correctionAngle) > 30) {
            done = true
            return
        }
        mDrive.arcadeDrive(power * direction, correctionAngle * 0.01 * direction.toDouble())
        //        System.out.println("yaw: " + yaw);
        println("correctionAngle: " + correctionAngle)
        SmartDashboard.putNumber("distanceInAction", Math.abs(mDrive.getRightDistanceInches()) - otherStart)
    }

    override fun done() {
        mDrive.setOpenLoop(DriveSignal.NEUTRAL)
        mDrive.highGear = !mDrive.highGear
        mDrive.highGear = !mDrive.highGear
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
