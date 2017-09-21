package org.usfirst.frc.team4099.lib.util

import org.usfirst.frc.team4099.robot.Constants

import java.io.BufferedReader
import java.io.FileNotFoundException
import java.io.InputStreamReader
import java.net.HttpURLConnection
import java.net.URL

object Utils {
    /**
     * Limits the given input to the given magnitude.
     * @param v         value to limit
     * @param limit     limited magnitude
     * @return          the limited value
     */
    fun limit(v: Double, limit: Double): Double {
        if (Math.abs(v) < limit)
            return v
        return if (v < 0)
            -limit
        else
            limit
    }

    fun diff(current: Double, prev: Double): Double {
        return Math.abs(current - prev)
    }

    fun around(value: Double, around: Double, tolerance: Double): Boolean {
        return diff(value, around) <= tolerance
    }

    fun sameSign(new_: Double, old_: Double): Boolean {
        return new_ >= 0 && old_ >= 0 || new_ <= 0 && old_ <= 0
    }

    fun sign(value: Double): Int {
        return if (value >= 0) 1 else -1
    }

    private fun getHTML(urlToRead: String): String {
        try {
            val result = StringBuilder()
            val url = URL(urlToRead)
            val conn = url.openConnection() as HttpURLConnection
            conn.connectTimeout = Constants.Autonomous.CONNECTION_TIMEOUT_MILLIS
            conn.readTimeout = Constants.Autonomous.CONNECTION_TIMEOUT_MILLIS
            conn.requestMethod = "GET"
            val rd = BufferedReader(InputStreamReader(conn.inputStream))
            var line: String? = rd.readLine()
            while (line != null) {
                result.append(line)
                line = rd.readLine()
            }
            rd.close()
            return result.toString()
        } catch (e: Exception) {
            return "-1"
        }

    }

    private fun getNumbersFromString(htmlOutput: String): DoubleArray {
        val htmlArray = htmlOutput.split(",".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()
        val anglesArray = DoubleArray(htmlArray.size)
        for (i in htmlArray.indices) {
            anglesArray[i] = java.lang.Double.parseDouble(htmlArray[i])
        }
        return anglesArray
    }

    @Throws(FileNotFoundException::class)
    private fun getNumbersFromUrl(urlToReadFromUdoo: String): DoubleArray {
        for (i in 0 until Constants.Autonomous.NUMBER_OF_TRIES) {
            val html = getHTML(Constants.Autonomous.UDOO_ADDRESS + urlToReadFromUdoo)
            println("HTML: " + html)
            if (html != "-1") {
                return getNumbersFromString(html)
            }
        }
        throw FileNotFoundException()
    }

    val gearLocation: GearVision
        @Throws(FileNotFoundException::class)
        get() {
            val gearVision = getNumbersFromUrl("get_gear")
            return GearVision(gearVision[0], gearVision[1] * 39.37)
        }

    val liftLocation: LiftVision
        @Throws(FileNotFoundException::class)
        get() {
            val liftVision = getNumbersFromUrl("get_lift")
            return LiftVision(liftVision[0], liftVision[1], liftVision[2] * 39.37)
        }

    fun getAverageFromList(list: List<Double>): Double {
        var total = 0.0
        for (d in list) {
            total += d
        }
        return total / list.size
    }
}
