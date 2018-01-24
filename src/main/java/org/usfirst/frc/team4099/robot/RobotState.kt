package org.usfirst.frc.team4099.robot

import org.usfirst.frc.team4099.lib.util.*
import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D
import org.usfirst.frc.team4099.lib.util.math.Twist2D
//import org.usfirst.frc.team4099.GoalTracker.TrackReport
//import org.usfirst.frc.team4099.vision.TargetInfo


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

import kotlin.collections.List
import kotlin.collections.Map
import java.util.Optional
import kotlin.collections.ArrayList

//this class has a number of variables that may or may not be necessary, including the camera ADJUSTMENTS
class RobotState() {
    companion object {
        private val OBSEVERVATION_BUFFER_SIZE: Int = 100
        //private val VEHICLE_TO_CAMERA: RigidTransform2D
        private val inst: RobotState = RobotState()

        fun getInstance(): RobotState {
            return inst
        }
    }

    init {
        reset(0.0,RigidTransform2D())
    }

    private var fieldToVehicle: InterpolatingTreeMap<InterpolatingDouble, RigidTransform2D> = InterpolatingTreeMap(OBSEVERVATION_BUFFER_SIZE)
    var predictedVehicleVel: Twist2D = Twist2D.identity()
    var measuredVehicleVel: Twist2D = Twist2D.identity()
    var distDriven: Double = 0.0

    @Synchronized fun reset(startT: Double, initFieldToVehicle: RigidTransform2D) {
        fieldToVehicle = InterpolatingTreeMap(OBSEVERVATION_BUFFER_SIZE)
        fieldToVehicle.put(InterpolatingDouble(startT), initFieldToVehicle)
        predictedVehicleVel = Twist2D.identity()
        measuredVehicleVel = Twist2D.identity()
        distDriven = 0.0
    }

    @Synchronized fun resetDistDriven() {
        distDriven = 0.0
    }

    @Synchronized fun getFieldToVehicle(ts: Double): RigidTransform2D {
        return fieldToVehicle.getInterpolated(InterpolatingDouble(ts))
    }

    @Synchronized fun getLatestFieldToVehicle(): Map.Entry<InterpolatingDouble, RigidTransform2D> {
        return fieldToVehicle.lastEntry()
    }

    @Synchronized fun getPredictedFieldToVehicle(lookaheadTime: Double): RigidTransform2D {
        return getLatestFieldToVehicle().value.transformBy(RigidTransform2D.exp(predictedVehicleVel.scaled(lookaheadTime)))
    }

    //@Synchronized fun getFieldToCamera(ts: Double)

    //@Synchronized fun getCaptureTimeFieldToGoal(): List<RigidTransform2D>

    @Synchronized fun addFieldToVehicleObservation(ts: Double, obs: RigidTransform2D) {
        fieldToVehicle.put(InterpolatingDouble(ts), obs)
    }

    @Synchronized fun addObservations(ts: Double, measVel: Twist2D, predVel: Twist2D) {
        addFieldToVehicleObservation(ts, Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().value, measVel))
        measuredVehicleVel = measVel
        predictedVehicleVel = predVel
    }

    //fun addVisionUpdate(ts: Double, visionUpdate: List<TargetInfo>)

    //@Synchronized fun resetVision()

    @Synchronized fun generateOdometryFromSensors(lEncoderDDist: Double, rEncoderDDist: Double, curGyroAngle: Rotation2D): Twist2D {
        val lastMeasure: RigidTransform2D = getLatestFieldToVehicle().value
        val delta: Twist2D = Kinematics.forwardKinematics(lastMeasure.getRotation(), lEncoderDDist, rEncoderDDist, curGyroAngle)
        distDriven += delta.dx()
        return delta
    }

    fun outputToSmartDashboard() {
        val odometry: RigidTransform2D = getLatestFieldToVehicle().value
        SmartDashboard.putNumber("robot_pose_x", odometry.getTranslation().x())
        SmartDashboard.putNumber("robot_pose_y", odometry.getTranslation().y())
        SmartDashboard.putNumber("robot_pose_theta", odometry.getRotation().degrees)
        SmartDashboard.putNumber("robot velocity", measuredVehicleVel.dx())
    }
}
