package org.usfirst.frc.team4099.robot.loops

import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team4099.lib.util.CrashTrackingRunnable
import org.usfirst.frc.team4099.robot.Constants
import java.util.*

class Looper(private val name: String) {

    private var running: Boolean = false

    private val mNotifier: Notifier
    private val taskRunningLock = Any()

    private val mLoops: MutableList<Loop> = ArrayList()

    private var timestamp = 0.0
    private var dt = 0.0

    // define the thread (runnable) that will be repeated continuously
    private val runnable = object : CrashTrackingRunnable() {
        override fun runCrashTracked() {
            synchronized(taskRunningLock) {
                if (running) {
                    val now = Timer.getFPGATimestamp()
                    for (loop in mLoops) {
                        loop.onLoop()
                    }
                    dt = now - timestamp
                    timestamp = now
                }
            }
        }
    }

    init {
        mNotifier = Notifier(runnable)
        running = false
    }

    @Synchronized
    fun register(loop: Loop) {
        synchronized(taskRunningLock) {
            mLoops.add(loop)
        }
    }

    @Synchronized
    fun start() {
        if (!running) {
            println("Starting looper: " + name)
            synchronized(taskRunningLock) {
                timestamp = Timer.getFPGATimestamp()
                for (loop in mLoops) {
                    loop.onStart()
                }

                running = true
            }
            mNotifier.startPeriodic(kPeriod)
        }
    }

    @Synchronized
    fun stop() {
        if (running) {
            println("Stopping looper: " + name)
            mNotifier.stop()

            synchronized(taskRunningLock) {
                running = false

                for (loop in mLoops) {
                    println("Stopping " + loop) // give the loops a name
                    loop.onStop()
                }
            }
        }
    }

    fun outputToSmartDashboard() {
        SmartDashboard.putNumber("looper_dt ($name)", dt)
    }

    companion object {

        private val kPeriod = Constants.Loopers.LOOPER_DT
    }

}
