package org.usfirst.frc.team4099.robot.loops

import edu.wpi.first.wpilibj.CameraServer

/** Manages the shutting off of components and subsystems when at risk of brownout.
 * It does this through a multitude of steps:
 * 1. Constantly monitor the Battery voltage
 * 2.
 * 3.
 */
class CameraSwitcher private constructor() : Loop {
    enum class Camera {
        INTAKE_CAMERA,
        ELEVATOR_CAMERA
    }

    private val intakeCamera = CameraServer.getInstance().startAutomaticCapture(0)
    private val elevatorCamera = CameraServer.getInstance().startAutomaticCapture(1)
    private val server = CameraServer.getInstance().getServer()

    private var currentSource = Camera.ELEVATOR_CAMERA

    override fun onStart() {
        server.source = elevatorCamera
        currentSource = Camera.ELEVATOR_CAMERA
    }

    override fun onLoop() {

    }

    override fun onStop() {

    }

    fun switchCamera(camera: Camera) {
        when (camera) {
            Camera.INTAKE_CAMERA -> {
                server.source = intakeCamera
                currentSource = Camera.INTAKE_CAMERA
            }
            Camera.ELEVATOR_CAMERA -> {
                server.source = elevatorCamera
                currentSource = Camera.ELEVATOR_CAMERA
            }
        }
    }

    companion object {
        val instance = CameraSwitcher()
    }
}
