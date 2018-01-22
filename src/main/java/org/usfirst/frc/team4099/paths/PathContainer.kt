package org.usfirst.frc.team4099.paths

import org.usfirst.frc.team4099.lib.util.control.Path
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D

interface PathContainer {
    fun buildPath(): Path

    fun getStartPose(): RigidTransform2D

    fun isReversed(): Boolean
}