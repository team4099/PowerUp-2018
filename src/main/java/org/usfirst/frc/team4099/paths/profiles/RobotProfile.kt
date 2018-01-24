package org.usfirst.frc.team4099.paths.profiles

/**
 * Interface that holds all the corrective values for how each robot actually drives.
 *
 * @see PathAdapter
 */

interface RobotProfile{
    //red
    public fun getRedSwitchXCorrection(): Double
    public fun getRedSwitchYCorrection(): Double
    public fun getRedRightPortalXOffset(): Double
    public fun getRedRightPortalYOffset(): Double


    // blue
    public fun getBlueSwitchXCorrection(): Double
    public fun getBlueSwitchYCorrection(): Double
    public fun getBlueRightPortalXOffset(): Double
    public fun getBlueRightPortalYOffset(): Double

}