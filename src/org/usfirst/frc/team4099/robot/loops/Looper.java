package org.usfirst.frc.team4099.robot.loops;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4099.lib.util.CrashTrackingRunnable;
import org.usfirst.frc.team4099.robot.Constants;

import java.util.ArrayList;
import java.util.List;

public class Looper {

    private static final double kPeriod = Constants.Loopers.LOOPER_DT;

    private boolean running;

    private final Notifier mNotifier;
    private final Object taskRunningLock = new Object();

    private final List<Loop> mLoops;

    private double timestamp = 0;
    private double dt = 0;

    private String name;

    // define the thread (runnable) that will be repeated continuously
    private final CrashTrackingRunnable runnable = new CrashTrackingRunnable() {
        @Override
        public void runCrashTracked() {
            synchronized (taskRunningLock) {
                if (running) {
                    double now = Timer.getFPGATimestamp();
                    for (Loop loop : mLoops) {
                        loop.onLoop();
                    }
                    dt = now - timestamp;
                    timestamp = now;
                }
            }
        }
    };

    public Looper(String name) {
        mNotifier = new Notifier(runnable);
        running = false;
        mLoops = new ArrayList<>();
        this.name = name;
    }

    public synchronized void register(Loop loop) {
        synchronized (taskRunningLock) {
            mLoops.add(loop);
        }
    }

    public synchronized void start() {
        if (!running) {
            System.out.println("Starting looper: " + name);
            synchronized (taskRunningLock) {
                timestamp = Timer.getFPGATimestamp();
                for (Loop loop : mLoops) {
                    loop.onStart();
                }

                running = true;
            }
            mNotifier.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop() {
        if (running) {
            System.out.println("Stopping looper: " + name);
            mNotifier.stop();

            synchronized (taskRunningLock) {
                running = false;

                for (Loop loop : mLoops) {
                    System.out.println("Stopping " + loop); // give the loops a name
                    loop.onStop();
                }
            }
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("looper_dt (" + name + ")", dt);
    }

}
