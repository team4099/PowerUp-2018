package org.usfirst.frc.team4099.lib.util;

public abstract class CrashTrackingRunnable implements Runnable {

    @Override
    public final void run() {
        try {
            runCrashTracked();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash("CrTrRu.run", t);
            throw t;
        }
    }

    public abstract void runCrashTracked();
}
