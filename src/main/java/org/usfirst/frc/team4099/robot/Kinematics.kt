package org.usfirst.frc.team4099.robot

import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Twist2D

object Kinematics {
    private val kEpsilon: Double = 1E-9

    fun forwardKinematics(lwd: Double, rwd: Double): Twist2D {
        val dv: Double = (rwd - lwd) / 2 * Constants.Wheels.TRACK_SCRUB_FACTOR
        val drot: Double = dv * 2 / Constants.Wheels.TRACK_WIDTH_INCHES
        return forwardKinematics(lwd,rwd,drot)
    }

    fun forwardKinematics(lwd: Double, rwd: Double, drotrads: Double): Twist2D {
        val dx: Double = (lwd + rwd / 2.0)
        return Twist2D(dx, 0.0, drotrads)
    }

    fun forwardKinematics(prevhead: Rotation2D, lwd: Double, rwd: Double, curHead: Rotation2D): Twist2D {
        return forwardKinematics(lwd, rwd, prevhead.inverse().rotateBy(curHead).radians)
    }

    fun integrateForwardKinematics(curpose: RigidTransform2D, lwd: Double, rwd: Double, curHead: Rotation2D): RigidTransform2D {
        val withGyro: Twist2D = forwardKinematics(curpose.getRotation(), lwd, rwd, curHead)
        return integrateForwardKinematics(curpose, withGyro)
    }

    fun integrateForwardKinematics(curpose: RigidTransform2D, forkin: Twist2D): RigidTransform2D {
        return curpose.transformBy(RigidTransform2D.exp(forkin))
    }

    class DriveVelocity(l: Double, r: Double) {
        val left: Double = l
        val right: Double = r
    }

    fun inverseKinematics(vel: Twist2D): DriveVelocity {
        if (Math.abs(vel.dtheta()) < kEpsilon) {
            return DriveVelocity(vel.dx(), vel.dx())
        }
        val dv: Double = Constants.Wheels.TRACK_WIDTH_INCHES * vel.dtheta() / (2 * Constants.Wheels.TRACK_SCRUB_FACTOR)
        return DriveVelocity(vel.dx() - dv, vel.dx() + dv)
    }
}