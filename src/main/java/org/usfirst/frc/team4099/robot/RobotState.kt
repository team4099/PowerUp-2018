package org.usfirst.frc.team4099.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import jdk.nashorn.internal.runtime.regexp.joni.constants.TargetInfo
import java.util.ArrayList
import org.usfirst.frc.team4099.lib.util.Twist2D



class RobotState private constructor(){

    private val instance_ : RobotState = RobotState()
    public fun getInstance() : RobotState{
        return instance_
    }

    private final val kObservationBufferSize : Int = 100

    private final var kVehicleToCamera : kVehicleToCamera = RigidTransform2d(
            Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset), Rotation2d())

    // FPGATimestamp -> RigidTransform2d or Rotation2d
    private var field_to_vehicle_: InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d>? = null
    private var vehicle_velocity_predicted_: Twist2D = null
    private var vehicle_velocity_measured_: Twist2D = null
    private var distance_driven_: Double = 0.toDouble()
    private var goal_tracker_: GoalTracker? = null
    private var camera_pitch_correction_: Rotation2d? = null
    private var camera_yaw_correction_: Rotation2d? = null
    private var differential_height_: Double = 0.toDouble()
    private var cached_shooter_aiming_params_: ShooterAimingParameters? = null

    private fun RobotState(): ??? {
        reset(0.0, RigidTransform2d())
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    @Synchronized
    fun reset(start_time: Double, initial_field_to_vehicle: RigidTransform2d) {
        field_to_vehicle_ = InterpolatingTreeMap(kObservationBufferSize)
        field_to_vehicle_!!.put(InterpolatingDouble(start_time), initial_field_to_vehicle)
        vehicle_velocity_predicted_ = Twist2D.identity()
        vehicle_velocity_measured_ = Twist2D.identity()
        goal_tracker_ = GoalTracker()
        camera_pitch_correction_ = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees)
        camera_yaw_correction_ = Rotation2d.fromDegrees(-Constants.kCameraYawAngleDegrees)
        differential_height_ = Constants.kBoilerTargetTopHeight - Constants.kCameraZOffset
        distance_driven_ = 0.0
    }

    @Synchronized
    fun resetDistanceDriven() {
        distance_driven_ = 0.0
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    @Synchronized
    fun getFieldToVehicle(timestamp: Double): RigidTransform2d {
        return field_to_vehicle_!!.getInterpolated(InterpolatingDouble(timestamp))
    }

    @Synchronized
    fun getLatestFieldToVehicle(): Entry<InterpolatingDouble, RigidTransform2d> {
        return field_to_vehicle_!!.lastEntry()
    }

    @Synchronized
    fun getPredictedFieldToVehicle(lookahead_time: Double): RigidTransform2d {
        return getLatestFieldToVehicle().value
                .transformBy(RigidTransform2d.exp(vehicle_velocity_predicted_!!.scaled(lookahead_time)))
    }

    @Synchronized
    fun getFieldToCamera(timestamp: Double): RigidTransform2d {
        return getFieldToVehicle(timestamp).transformBy(kVehicleToCamera)
    }

    @Synchronized
    fun getCaptureTimeFieldToGoal(): List<RigidTransform2d> {
        val rv = ArrayList<E>()
        for (report in goal_tracker_!!.getTracks()) {
            rv.add(RigidTransform2d.fromTranslation(report.field_to_goal))
        }
        return rv
    }

    @Synchronized
    fun addFieldToVehicleObservation(timestamp: Double, observation: RigidTransform2d) {
        field_to_vehicle_!!.put(InterpolatingDouble(timestamp), observation)
    }

    @Synchronized
    fun addObservations(timestamp: Double, measured_velocity: Twist2D,
                        predicted_velocity: Twist2D) {
        addFieldToVehicleObservation(timestamp,
                Kinematics.integrateForwardKinematics(getLatestFieldToVehicle().value, measured_velocity))
        vehicle_velocity_measured_ = measured_velocity
        vehicle_velocity_predicted_ = predicted_velocity
    }

    fun addVisionUpdate(timestamp: Double, vision_update: List<TargetInfo>?) {
        val field_to_goals = ArrayList<E>()
        val field_to_camera = getFieldToCamera(timestamp)
        if (!(vision_update == null || vision_update.isEmpty())) {
            for (target in vision_update) {
                val ydeadband = if (target.getY() > -Constants.kCameraDeadband && target.getY() < Constants.kCameraDeadband)
                    0.0
                else
                    target.getY()

                // Compensate for camera yaw
                val xyaw = target.getX() * camera_yaw_correction_!!.cos() + ydeadband * camera_yaw_correction_!!.sin()
                val yyaw = ydeadband * camera_yaw_correction_!!.cos() - target.getX() * camera_yaw_correction_!!.sin()
                val zyaw = target.getZ()

                // Compensate for camera pitch
                val xr = zyaw * camera_pitch_correction_!!.sin() + xyaw * camera_pitch_correction_!!.cos()
                val zr = zyaw * camera_pitch_correction_!!.cos() - xyaw * camera_pitch_correction_!!.sin()

                // find intersection with the goal
                if (zr > 0) {
                    val scaling = differential_height_ / zr
                    val distance = Math.hypot(xr, yyaw) * scaling + Constants.kBoilerRadius
                    val angle = Rotation2d(xr, yyaw, true)
                    field_to_goals.add(field_to_camera
                            .transformBy(RigidTransform2d
                                    .fromTranslation(Translation2d(distance * angle.cos(), distance * angle.sin())))
                            .getTranslation())
                }
            }
        }
        synchronized(this) {
            goal_tracker_!!.update(timestamp, field_to_goals)
        }
    }

    @Synchronized
    fun getCachedAimingParameters(): Optional<ShooterAimingParameters> {
        return if (cached_shooter_aiming_params_ == null) Optional.empty() else Optional.of(cached_shooter_aiming_params_)
    }

    @Synchronized
    fun getAimingParameters(): Optional<ShooterAimingParameters> {
        val reports = goal_tracker_!!.getTracks()
        if (!reports.isEmpty()) {
            val report = reports.get(0)
            val robot_to_goal = getLatestFieldToVehicle().value.getTranslation().inverse()
                    .translateBy(report.field_to_goal)
            val robot_to_goal_rotation = Rotation2d
                    .fromRadians(Math.atan2(robot_to_goal.y(), robot_to_goal.x()))

            val params = ShooterAimingParameters(robot_to_goal.norm(), robot_to_goal_rotation,
                    report.latest_timestamp, report.stability)
            cached_shooter_aiming_params_ = params

            return Optional.of(params)
        } else {
            return Optional.empty()
        }
    }

    @Synchronized
    fun resetVision() {
        goal_tracker_!!.reset()
        cached_shooter_aiming_params_ = null
    }

    @Synchronized
    fun generateOdometryFromSensors(left_encoder_delta_distance: Double,
                                    right_encoder_delta_distance: Double, current_gyro_angle: Rotation2d): Twist2D {
        val last_measurement = getLatestFieldToVehicle().value
        val delta = Kinematics.forwardKinematics(last_measurement.getRotation(),
                left_encoder_delta_distance, right_encoder_delta_distance, current_gyro_angle)
        distance_driven_ += delta.dx
        return delta
    }

    @Synchronized
    fun getDistanceDriven(): Double {
        return distance_driven_
    }

    @Synchronized
    fun getPredictedVelocity(): Twist2D {
        return vehicle_velocity_predicted_
    }

    @Synchronized
    fun getMeasuredVelocity(): Twist2D {
        return vehicle_velocity_measured_
    }

    fun outputToSmartDashboard() {
        val odometry = getLatestFieldToVehicle().value
        SmartDashboard.putNumber("robot_pose_x", odometry.getTranslation().x())
        SmartDashboard.putNumber("robot_pose_y", odometry.getTranslation().y())
        SmartDashboard.putNumber("robot_pose_theta", odometry.getRotation().getDegrees())
        SmartDashboard.putNumber("robot velocity", vehicle_velocity_measured_!!.dx)
        val poses = getCaptureTimeFieldToGoal()
        for (pose in poses) {
            // Only output first goal
            SmartDashboard.putNumber("goal_pose_x", pose.getTranslation().x())
            SmartDashboard.putNumber("goal_pose_y", pose.getTranslation().y())
            break
        }
        val aiming_params = getCachedAimingParameters()
        if (aiming_params.isPresent()) {
            SmartDashboard.putNumber("goal_range", aiming_params.get().getRange())
            SmartDashboard.putNumber("goal_theta", aiming_params.get().getRobotToGoal().getDegrees())
        } else {
            SmartDashboard.putNumber("goal_range", 0.0)
            SmartDashboard.putNumber("goal_theta", 0.0)
        }
    }
}