package org.usfirst.frc.team4099.lib.util;

import org.usfirst.frc.team4099.robot.Constants;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.List;

public class Utils {
    /**
     * Limits the given input to the given magnitude.
     * @param v         value to limit
     * @param limit     limited magnitude
     * @return          the limited value
     */
    public static double limit(double v, double limit) {
        if (Math.abs(v) < limit)
            return v;
        if (v < 0)
            return -limit;
        else
            return limit;
    }

    public static double diff(double current, double prev) {
        return Math.abs(current - prev);
    }

    public static boolean around(double value, double around, double tolerance) {
        return diff(value, around) <= tolerance;
    }

    public static boolean sameSign(double new_, double old_) {
        return (new_ >= 0 && old_ >= 0) || (new_ <= 0 && old_ <= 0);
    }

    public static int sign(double value) {
        return (value >= 0)? 1 : -1;
    }

    private static String getHTML(String urlToRead) {
        try {
            StringBuilder result = new StringBuilder();
            URL url = new URL(urlToRead);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setConnectTimeout(Constants.Autonomous.CONNECTION_TIMEOUT_MILLIS);
            conn.setReadTimeout(Constants.Autonomous.CONNECTION_TIMEOUT_MILLIS);
            conn.setRequestMethod("GET");
            BufferedReader rd = new BufferedReader(new InputStreamReader(conn.getInputStream()));
            String line;
            while((line = rd.readLine()) != null) {
                result.append(line);
            }
            rd.close();
            return result.toString();
        } catch(Exception e) {
            return "-1";
        }
    }

    private static double[] getNumbersFromString(String htmlOutput) {
        String[] htmlArray = htmlOutput.split(",");
        double[] anglesArray = new double[htmlArray.length];
        for(int i = 0; i < htmlArray.length; i++) {
            anglesArray[i] = Double.parseDouble(htmlArray[i]);
        }
        return anglesArray;
    }

    public static double[] getNumbersFromUrl(String urlToReadFromUdoo) throws FileNotFoundException {
        for (int i = 0; i < Constants.Autonomous.NUMBER_OF_TRIES; i++) {
            String html = getHTML(Constants.Autonomous.UDOO_ADDRESS + urlToReadFromUdoo);
            System.out.println("HTML: " + html);
            if (!html.equals("-1")) {
                return getNumbersFromString(html);
            }
        }
        throw new FileNotFoundException();
    }

    public static GearVision getGearLocation() throws FileNotFoundException {
        double[] gearVision = getNumbersFromUrl("get_gear");
        return new GearVision(gearVision[0], gearVision[1] * 39.37);
    }

    public static LiftVision getLiftLocation() throws FileNotFoundException {
        double[] liftVision = getNumbersFromUrl("get_lift");
        return new LiftVision(liftVision[0], liftVision[1], liftVision[2] * 39.37);
    }

    public static double getAverageFromList(List<Double> list) {
        double total = 0;
        for(Double d : list) {
            total += d;
        }
        return total / list.size();
    }
}
