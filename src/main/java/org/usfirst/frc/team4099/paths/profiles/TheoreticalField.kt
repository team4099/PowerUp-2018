package org.usfirst.frc.team4099.paths.profiles

/**
 * Created by O on 1/24/2018.
 */

class TheoreticalField : FieldProfile{
    override fun getRedRightPortalToExchange(): Double {
        return 13.237
    }

    override fun getRedLeftPortalToExchange(): Double {
        return 7.237
    }

    override fun getRedBaseLineToAutoLine(): Double {
        return 10.0
    }

    override fun getRedBaseLineToSwitch(): Double {
        return 11.667
    }

    override fun getRedBaseLineToBackOfSwitch(): Double {
        return 16.333
    }

    override fun getRedBaseLineToCenterOfPlatform(): Double {
        return 27.0
    }


    //blue

    override fun getBlueRightPortalToExchange(): Double {
        return 13.237
    }

    override fun getBlueLeftPortalToExchange(): Double {
        return 7.237
    }

    override fun getBlueBaseLineToAutoLine(): Double {
        return 10.0
    }

    override fun getBlueBaseLineToSwitch(): Double {
        return 11.667
    }

    override fun getBlueBaseLineToBackOfSwitch(): Double {
        return 16.333
    }

    override fun getBlueBaseLineToCenterOfPlatform(): Double {
        return 27.0
    }

}