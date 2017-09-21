package org.usfirst.frc.team4099.lib.util

/**
 * Created by plato2000 on 3/4/17.
 */
class LiftVision(offsetAngle: Double, turnAngle: Double, val distance: Double) {

    private val offsetAngle: Rotation2D = Rotation2D.fromDegrees(offsetAngle)
    private val turnAngle: Rotation2D = Rotation2D.fromDegrees(turnAngle)

    fun getOffsetAngle(): Rotation2D {
        return Rotation2D(offsetAngle)
    }

    fun getTurnAngle(): Rotation2D {
        return Rotation2D(turnAngle)
    }

}
