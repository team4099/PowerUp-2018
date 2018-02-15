package main.java.org.usfirst.frc.team4099.lib.util

import kotlinx.serialization.*
import kotlinx.serialization.internal.SerialClassDescImpl
import org.usfirst.frc.team4099.auto.modes.AutoModeBase

@Serializable
data class AutoModeCreator(val dashboardName: String, val creator: (startPos: String, ownershipConfig: String, delay: Double) -> AutoModeBase)