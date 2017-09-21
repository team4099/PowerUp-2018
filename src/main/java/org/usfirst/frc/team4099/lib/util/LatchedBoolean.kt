package org.usfirst.frc.team4099.lib.util

/**
 * An iterative boolean latch.
 *
 * Returns true once if and only if the value of newValue
 * changes from false to true.
 *
 */

class LatchedBoolean {
    private var mLast = false

    fun update(newValue: Boolean): Boolean {
        var ret = false
        if (newValue && !mLast)
            ret = true

        mLast = newValue
        return ret
    }
}
