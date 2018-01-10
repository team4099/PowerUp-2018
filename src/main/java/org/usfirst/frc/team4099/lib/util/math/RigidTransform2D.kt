package org.usfirst.frc.team4099.lib.util.math

import org.usfirst.frc.team4099.lib.util.Interpolable
import org.usfirst.frc.team4099.lib.util.Utils.epsilonEquals

class RigidTransform2D: Interpolable<RigidTransform2D> {

    protected var translation_: Translation2D = Translation2D()
    protected var rotation_: Rotation2D = Rotation2D()

    @JvmOverloads constructor(t: Translation2D = Translation2D(), r: Rotation2D = Rotation2D()){
        translation_ = t
        rotation_ = r
    }

    constructor(other: RigidTransform2D) {
        translation_ = other.translation_
        rotation_ = other.rotation_
    }

    fun getTranslation(): Translation2D {
        return translation_
    }

    fun getRotation(): Rotation2D {
        return rotation_
    }

    fun setTranslation(tran: Translation2D) {
        translation_ = tran
    }

    fun setRotation(rot: Rotation2D) {
        rotation_ = rot
    }

    fun transformBy(other: RigidTransform2D): RigidTransform2D {
        return RigidTransform2D(translation_.translateBy(other.translation_.rotateBy(rotation_)), rotation_.rotateBy(other.rotation_))
    }

    fun inverse(): RigidTransform2D {
        val rot_inv: Rotation2D = rotation_.inverse()
        return RigidTransform2D(translation_.inverse().rotateBy(rot_inv), rot_inv)
    }

    fun normal(): RigidTransform2D {
        return RigidTransform2D(translation_, rotation_.normal())
    }

    fun intersection(other: RigidTransform2D): Translation2D {
        val oth_rot: Rotation2D = other.rotation_
        if (rotation_.isParallel(oth_rot)) {
            return Translation2D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)
        }
        if (Math.abs(rotation_.cos()) < Math.abs(oth_rot.cos())) {
            return intersectionInternal(this, other)
        } else {
            return intersectionInternal(other, this)
        }
    }

    fun isColinear(other: RigidTransform2D): Boolean {
        val twist: Twist2D = log(inverse().transformBy(other))
        return (epsilonEquals(twist.dy(), 0.0, kEpsilon) && epsilonEquals(twist.dtheta(), 0.0, kEpsilon))
    }

    fun intersectionInternal(a: RigidTransform2D, b: RigidTransform2D): Translation2D {
        val a_r: Rotation2D = a.getRotation()
        val b_r: Rotation2D = b.getRotation()
        val a_t: Translation2D = a.getTranslation()
        val b_t: Translation2D = b.getTranslation()

        val tan_b: Double = b_r.tan()
        val t: Double = ((a_t.x() - b_t.x())*tan_b + b_t.y() - a_t.y())/(a_r.sin() - a_r.cos() * tan_b)
        return a_t.translateBy(a_r.toTranslation().scale(t))
    }

    override fun interpolate(other: RigidTransform2D, x: Double): RigidTransform2D {
        if (x<=0) {
            return RigidTransform2D(this)
        } else if (x>=1) {
            return RigidTransform2D(other)
        }
        val twist:Twist2D = RigidTransform2D.log(inverse().transformBy(other));
        return transformBy(RigidTransform2D.exp(twist.scaled(x)))
    }

    override fun toString(): String {
        return "T: "+translation_.toString()+", R: "+rotation_.toString()
    }

    companion object {
        protected val kEpsilon = 1E-9

        fun exp(delta: Twist2D): RigidTransform2D {
            val sin_t: Double = Math.sin(delta.dtheta())
            val cos_t: Double = Math.cos(delta.dtheta())
            val s: Double
            val c: Double

            if(Math.abs(delta.dtheta())< kEpsilon) {
                s = 1.0 - 1.0 / 6.0 * delta.dtheta() * delta.dtheta()
                c = 0.5 * delta.dtheta() - 1.0 / 24.0 * delta.dtheta() * delta.dtheta() * delta.dtheta()
            } else {
                s = sin_t / delta.dtheta()
                c = (1.0-cos_t) / delta.dtheta()
            }
            return RigidTransform2D(Translation2D(delta.dx() * s - delta.dy() * c, delta.dx() * c + delta.dy() * s), Rotation2D(cos_t, sin_t, false ))
        }

        fun log(transform: RigidTransform2D): Twist2D {
            val dtheta: Double = transform.rotation_.radians
            val half_dtheta: Double = 0.5 * dtheta
            val cos_sub_1: Double = transform.rotation_.radians - 1.0
            var halfth_mult_tanhalfdth: Double
            if (Math.abs(cos_sub_1) < kEpsilon) {
                halfth_mult_tanhalfdth = 1 - 1 / 12 * dtheta * dtheta
            } else {
                halfth_mult_tanhalfdth = -(half_dtheta * transform.rotation_.sin())/cos_sub_1
            }
            val trans_part: Translation2D = transform.translation_.rotateBy(Rotation2D(halfth_mult_tanhalfdth, -half_dtheta, false))
            return Twist2D(trans_part.x(), trans_part.y(), dtheta)
        }
    }
}