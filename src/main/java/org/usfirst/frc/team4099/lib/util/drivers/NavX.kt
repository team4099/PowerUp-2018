package org.usfirst.frc.team4099.lib.util.drivers

import edu.wpi.first.wpilibj.SPI

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase
import com.kauailabs.navx.frc.AHRS
import com.kauailabs.navx.frc.ITimestampedDataSubscriber

import org.usfirst.frc.team4099.lib.util.math.Rotation2D

open class NavX(spiPortID: SPI.Port) {

    protected var aHRS: AHRS = AHRS(spiPortID, 200.toByte())
    protected var angleAdjustment: Rotation2D = Rotation2D()
    protected var yawDegrees: Double = 0.0
    protected var yawRateDegreesPerSecond: Double = 0.0
    protected val invalidTimestamp: Long = -1
    protected var lastSensorTimestampMS: Long = invalidTimestamp

    @Synchronized fun reset() {
        aHRS.reset()
        resetState()
    }

    @Synchronized fun zeroYaw() {
        aHRS.zeroYaw()
        resetState()
    }

    private fun resetState() {
        lastSensorTimestampMS = invalidTimestamp
        yawDegrees = 0.0
        yawRateDegreesPerSecond = 0.0
    }

    @Synchronized fun getRawYawDeg(): Double {
        return yawDegrees
    }

    fun getYaw(): Rotation2D {
        return angleAdjustment.rotateBy(Rotation2D.fromDegrees(getRawYawDeg()))
    }

    fun getYawRateDegPerSec(): Double {
        return yawRateDegreesPerSecond
    }

    fun getYawRateRadPerSec(): Double {
        return 180.0/Math.PI * getYawRateDegPerSec()
    }

    fun getRawAccelX(): Float {
        return aHRS.rawAccelX
    }

    init {
        resetState()
        aHRS.registerCallback(Callback(), null)
    }

    inner class Callback: ITimestampedDataSubscriber {
        override fun timestampedDataReceived(system_timestamp: Long, sensor_timestamp: Long, update: AHRSUpdateBase?, context: Any?) {
            synchronized(this@NavX) {
                if (lastSensorTimestampMS != invalidTimestamp && lastSensorTimestampMS < sensor_timestamp) {
                    yawRateDegreesPerSecond = 1000.0 * (-yawDegrees - update!!.yaw) / ((sensor_timestamp - lastSensorTimestampMS).toDouble())
                }
                lastSensorTimestampMS = sensor_timestamp
                yawDegrees = -update!!.yaw.toDouble()

            }
        }
    }
}