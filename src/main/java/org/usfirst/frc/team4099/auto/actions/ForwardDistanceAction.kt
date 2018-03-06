package org.usfirst.frc.team4099.auto.actions

import edu.wpi.first.wpilibj.Timer
import org.usfirst.frc.team4099.lib.util.Utils
import org.usfirst.frc.team4099.robot.subsystems.Drive

/**
 * Created by Oksana on 2/16/2017.
 */
class ForwardDistanceAction(initInchesToMove: Double) : Action {
    private val mDrive: Drive = Drive.instance
    private val inchesToMove: Double = Math.abs(initInchesToMove)
    private var startDist: Double = 0.toDouble()
    private val direction: Int
    private var power: Double = 0.toDouble()
    private var startAngle: Double = 0.toDouble()
    private var resetGyro: Boolean = false
    private var done: Boolean = false

    constructor(inchesToMove: Double, slowMode: Boolean, resetGyro: Boolean) : this(inchesToMove) {
        if (slowMode) {
            this.power = .35
        }
        this.resetGyro = resetGyro
    }

    init {
        direction = this.inchesToMove.toInt() / this.inchesToMove.toInt()
        this.power = .5
    }

    override fun isFinished(): Boolean {
        return mDrive.getLeftDistanceInches() - startDist >= inchesToMove || done
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
        mDrive.arcadeDrive(-power * direction, correctionAngle * 0.07 * direction.toDouble())
        //        System.out.println("yaw: " + yaw);
        println("correctionAngle: " + correctionAngle)
    }

    override fun done() {
        //        mDrive.finishForward()
        println("------- END FORWARD -------")
    }

    override fun start() {
        if (resetGyro) {
            while (!Utils.around(mDrive.getAHRS()!!.yaw.toDouble(), 0.0, 1.0)) {
                mDrive.getAHRS()!!.zeroYaw()
            }
            Timer.delay(1.0)
        }
        startAngle = mDrive.getAHRS()!!.yaw.toDouble()
        println("------- NEW START AUTONOMOUS RUN -------")
        println("Starting angle: " + startAngle)
        startDist = mDrive.getLeftDistanceInches()
    }
}
