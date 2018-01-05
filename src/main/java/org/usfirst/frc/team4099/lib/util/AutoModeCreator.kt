package main.java.org.usfirst.frc.team4099.lib.util

import org.usfirst.frc.team4099.auto.modes.AutoModeBase

data class AutoModeCreator(val dashboardName: String, val creator: () -> AutoModeBase)