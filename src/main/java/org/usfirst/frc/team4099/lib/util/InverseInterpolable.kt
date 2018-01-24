package org.usfirst.frc.team4099.lib.util

interface InverseInterpolable<T> {
    fun inverseInterpolate(upper: T, query: T): Double
}