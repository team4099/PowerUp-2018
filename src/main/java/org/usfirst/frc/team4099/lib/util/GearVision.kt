package org.usfirst.frc.team4099.lib.util

/**
 * Created by plato2000 on 3/4/17.
 */
class GearVision(angleToGear: Double, distanceToGear: Double) {
    private val turnToGear: Rotation2D
    val distance: Double = 0.toDouble()

    init {
        var distanceToGear = distanceToGear
        turnToGear = Rotation2D.fromDegrees(angleToGear)
        distanceToGear = distanceToGear
    }

    val turnAngle: Rotation2D
        get() = Rotation2D(turnToGear)

}
