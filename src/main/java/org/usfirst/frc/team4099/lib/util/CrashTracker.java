package org.usfirst.frc.team4099.lib.util;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

/**
 * Tracks startup and caught crash events, logging them to a file
 * while doesn't roll over
 */

public class CrashTracker {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

    public static void logRobotStartup() {
        logMarker("Robot Startup");
    }

    public static void logRobotConstruction() {
        logMarker("Robot Constructed");
    }

    public static void logRobotInit() {
        logMarker("Robot Initialized");
    }

    public static void logAutoInit() {
        logMarker("Autonomous Initialized");
    }

    public static void logTeleopInit() {
        logMarker("Teleop Initialized");
    }

    public static void logDisabledInit() {
        logMarker("Disabled Initialized");
    }

    public static void logThrowableCrash(String function, Throwable throwable) {
        logMarker("Exception @ " + function, throwable);
    }

    public static void logMarker(String mark) {
        logMarker(mark, null);
    }

    private static void logMarker(String mark, Throwable nullableException) {
        try (FileWriter fw = new FileWriter(
                "/home/lvuser/crash_tracking.txt",
                true);
             BufferedWriter bw = new BufferedWriter(fw);
             PrintWriter out = new PrintWriter(bw)) {

            out.print(RUN_INSTANCE_UUID.toString());
            out.print(", ");
            out.print(mark);
            out.print(", ");
            out.print(new Date().toString());

            if (nullableException != null) {
                out.print(", ");
                nullableException.printStackTrace(out);
            }

            out.println();
        } catch (IOException ex) {
            ex.printStackTrace();
        }

    }
}
