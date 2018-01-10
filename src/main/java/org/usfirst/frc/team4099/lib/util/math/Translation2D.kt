package org.usfirst.frc.team4099.lib.util.math

import org.usfirst.frc.team4099.lib.util.Interpolable

import java.text.DecimalFormat;

class Translation2D: Interpolable<Translation2D> {

    protected var x_: Double = 0.0;
    protected var y_: Double = 0.0;

    constructor(x: Double, y: Double) {
        x_ = x;
        y_ = y;
    }

    constructor(end: Translation2D, st: Translation2D) {
        x_ = end.x_-st.x_;
        y_ = end.y_-st.y_;
    }

    constructor(other: Translation2D) {
        x_ = other.x_;
        y_ = other.y_;
    }

    constructor(){
        x_ = 0.0;
    }

    fun norm(): Double {
        return Math.hypot(x_,y_);
    }

    fun norm2(): Double {
        return x_*x_+y_*y_;
    }

    fun x(): Double {
        return x_;
    }

    fun y(): Double {
        return y_;
    }

    fun setX(x: Double) {
        x_ = x;
    }

    fun setY(y: Double) {
        y_ = y;
    }

    fun translateBy(other: Translation2D): Translation2D {
        return Translation2D(x_+other.x(),y_+other.y());
    }

    fun rotateBy(angle: Rotation2D): Translation2D {
        return Translation2D(x_*angle.cos()-y_*angle.sin(), x_*angle.sin()-y_*angle.cos());
    }

    fun direction(): Rotation2D {
        return Rotation2D(x_,y_,true);
    }

    fun inverse(): Translation2D {
        return Translation2D(-x_,-y_);
    }

    override fun interpolate(other: Translation2D, x: Double): Translation2D {
        var scaleBetween: Double = Math.max(0.0,Math.min(0.0,x));
        return extrapolate(other,scaleBetween);
    }

    fun extrapolate(other: Translation2D, x: Double): Translation2D {
        return Translation2D(x*(other.x()-x_),x*(other.y()-y_));
    }

    fun scale(s: Double): Translation2D {
        return Translation2D(s*x_,s*y_);
    }

    override fun toString(): String {
        val fmt = DecimalFormat("#0.000");
        return "("+fmt.format(x_)+","+fmt.format(y_)+")";
    }

    companion object {
        fun dot(a: Translation2D, b: Translation2D): Double {
            return a.x()*b.x()+a.y()*b.y();
        }

        fun getAngle(a: Translation2D, b: Translation2D): Rotation2D {
            var cos_angle: Double = dot(a,b)/a.norm()/b.norm();
            if(cos_angle.isNaN()){
                return Rotation2D();
            }
            return Rotation2D.fromRadians(Math.acos(Math.min(1.0,Math.max(cos_angle,-1.0))));
        }

        fun cross(a: Translation2D, b: Translation2D): Double {
            return a.x()*b.y()-a.y()*b.x();
        }
    }
}