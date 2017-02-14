package org.usfirst.frc.team4099.lib.util;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

public class Utils {
    private Utils() {}

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

    public static String getHTML(String urlToRead) throws Exception {
        try {
            StringBuilder result = new StringBuilder();
            URL url = new URL(urlToRead);
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setConnectTimeout(1000);
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

    public static int[] getNumbersFromString(String htmlOutput) {
        String[] htmlArray = htmlOutput.split(",");
        int[] anglesArray = new int[htmlArray.length];
        for(int i = 0; i < htmlArray.length; i++) {
            anglesArray[i] = Integer.parseInt(htmlArray[i]);
        }
        return anglesArray;
    }
}
