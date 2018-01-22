package org.usfirst.frc.team4099.paths.profiles

/**
 * Created by O on 1/22/2018.
 */

interface FieldProfile{
    // red
    public fun getRedRightPortalToExchange(): Double
    public fun getRedLeftPortalToExchange(): Double
    public fun getRedBaseLineToAutoLine(): Double
    public fun getRedBaseLineToSwitch(): Double
    public fun getRedBaseLineToBackOfSwitch(): Double
    public fun getRedBaseLineToCenterOfPlatform(): Double

    //blue
    public fun getBlueRightPortalToExchange(): Double
    public fun getBlueLeftPortalToExchange(): Double
    public fun getBlueBaseLineToAutoLine(): Double
    public fun getBlueBaseLineToSwitch(): Double
    public fun getBlueBaseLineToBackOfSwitch(): Double
    public fun getBlueBaseLineToCenterOfPlatform(): Double

}