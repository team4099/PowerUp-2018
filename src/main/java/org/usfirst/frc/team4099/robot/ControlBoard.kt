package org.usfirst.frc.team4099.robot

interface ControlBoard {
    val throttle: Double

    val turn: Double

    val toggleSlowMode: Boolean

    val quickTurn: Boolean

    val intakeUp: Boolean

    val intakeDown: Boolean

    val toggleIntake: Boolean

    val climber: Boolean

    val toggleIntakeClosed: Boolean

}
