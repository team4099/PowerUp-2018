package org.usfirst.frc.team4099.auto.actions

import org.usfirst.frc.team4099.robot.subsystems.Drive

class WaitForPathMarkerAction: Action {
    private var mDrive: Drive = Drive.instance
    private var mMarker: String

    constructor(marker: String) {
        mMarker = marker
    }

    override fun isFinished(): Boolean {
        return mDrive.hasPassedMarker(mMarker)
    }

    override fun update() {
    }

    override fun done() {
    }

    override fun start() {
    }
}