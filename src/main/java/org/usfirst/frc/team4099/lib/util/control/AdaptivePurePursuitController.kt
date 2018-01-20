package org.usfirst.frc.team4099.lib.util.control

import org.usfirst.frc.team4099.lib.util.math.RigidTransform2D
import org.usfirst.frc.team4099.lib.util.math.Rotation2D
import org.usfirst.frc.team4099.lib.util.math.Translation2D
import org.usfirst.frc.team4099.lib.util.math.Twist2D

/**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4 .pdf
 *
 * Basically, we find a spot on the path we'd like to follow and calculate the arc necessary to make us land on that
 * spot. The target spot is a specified distance ahead of us, and we look further ahead the greater our tracking error.
 * We also return the maximum speed we'd like to be going when we reach the target spot.
 */

class AdaptivePurePursuitController{

    companion object {
        private val kReallyBigNumber: Double = 1E6

        class Command{
            var delta: Twist2D = Twist2D.identity()
            var cross_track_error: Double
            var max_velocity: Double
            var end_velocity: Double
            var lookahead_point: Translation2D
            var remaining_path_length: Double

            constructor(delta:Twist2D, cross_track_error: Double, max_velocity: Double, end_velocity: Double,
                        lookahead_point: Translation2D, remaining_path_length: Double){
                this.delta = delta
                this.cross_track_error = cross_track_error
                this.max_velocity = max_velocity
                this.end_velocity = end_velocity
                this.lookahead_point = lookahead_point
                this.remaining_path_length = remaining_path_length
            }
        }

        class Arc{
            var center: Translation2D
            var radius: Double
            var length: Double

            constructor(pose: RigidTransform2D, point: Translation2D){
                center = getCenter(pose, point)
                radius = Translation2D(center, point).norm()
                length = getLength(pose, point, center, radius)
            }
        }

        fun getCenter(pose: RigidTransform2D, point: Translation2D): Translation2D{
            val poseToPointHalfway: Translation2D = pose.getTranslation().interpolate(point, 0.5)
            val normal: Rotation2D = pose.getTranslation().inverse().translateBy(poseToPointHalfway).direction().normal()
            val perpendicularBisector: RigidTransform2D = RigidTransform2D(poseToPointHalfway, normal)
            val normalFromPose: RigidTransform2D = RigidTransform2D(pose.getTranslation(), pose.getRotation().normal())

            if(normalFromPose.isColinear(perpendicularBisector.normal())){
                // Special case: center is poseToPointHalfway
                return poseToPointHalfway
            }
            return normalFromPose.intersection(perpendicularBisector)
        }

        fun getRadius(pose: RigidTransform2D, point: Translation2D): Double{
            var center: Translation2D = getCenter(pose, point)
            return Translation2D(center, point).norm()
        }

        fun getLength(pose: RigidTransform2D, point: Translation2D): Double{
            val radius: Double = getRadius(pose, point)
            val center: Translation2D = getCenter(pose, point)
            return getLength(pose, point, center, radius)
        }

        fun getLength(pose: RigidTransform2D, point: Translation2D, center: Translation2D, radius: Double):Double {
            if (radius< kReallyBigNumber){
                val centerToPoint: Translation2D = Translation2D(center, point)
                val centerToPose:Translation2D = Translation2D(center, pose.getTranslation())
                // If the point is behind the pose, we want the opposite of this angle. To determine if point is behind,
                // check the sign of the cross product between the normal vector and the vector from pose to point.
                val behind: Boolean = Math.signum(
                        Translation2D.cross(pose.getRotation().normal().toTranslation(),
                                Translation2D(pose.getTranslation(), point))) > 0.0
                val angle: Rotation2D = Translation2D.getAngle(centerToPose, centerToPoint)
                return radius * (if(behind) 2.0*Math.PI - Math.abs(angle.radians) else Math.abs(angle.radians))
            } else {
                return Translation2D(pose.getTranslation(), point).norm()
            }
        }

        fun getDirection (pose: RigidTransform2D, point: Translation2D): Int{
            var poseToPoint: Translation2D = Translation2D(pose.getTranslation(), point)
            var robot: Translation2D =  pose.getRotation().toTranslation()
            var cross: Double = robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x()
            return if(cross<0) -1 else 1  // If robot < pose then turn left
        }

    }

    var mPath: Path
    var mAtEndOfPath: Boolean = false
    final var mReversed: Boolean
    final var mLookahead: Lookahead

    constructor(path: Path, reversed: Boolean, lookahead: Lookahead){
        mPath = path
        mReversed = reversed
        mLookahead = lookahead
    }
    fun update(pose: RigidTransform2D):Command{
        if(mReversed){
            var pose = RigidTransform2D(pose.getTranslation(),
                    pose.getRotation().rotateBy(Rotation2D.fromRadians(Math.PI)))
        }
        val report: Path.Companion.TargetPointReport = mPath.getTargetPoint(pose.getTranslation(),mLookahead)
        if(isFinished()){
            //stop
            return Command(Twist2D.identity(), report.closestPointDistance,report.maxSpeed, 0.0,
                    report.lookaheadPoint, report.remainingPathDistance)
        }

        val arc: Arc = Arc(pose, report.lookaheadPoint)
        var scale_factor: Double = 1.0
        //Ensure we don't overshoot the end of the path (once the lookahead speed drops to 0)
        if (report.lookaheadPointSpeed < 1E-6 && report.remainingPathDistance < arc.length){
            scale_factor = Math.max (0.0, report.remainingPathDistance / arc.length)
            mAtEndOfPath = true
        } else{
            mAtEndOfPath = false
        }
        if (mReversed){
            scale_factor *=-1
        }

        return Command(
                Twist2D(scale_factor * arc.length, 0.0,
                        arc.length * getDirection(pose, report.lookaheadPoint)* Math.abs(scale_factor)/ arc.radius),
                report.closestPointDistance, report.maxSpeed,
                report.lookaheadPointSpeed * Math.signum(scale_factor), report.lookaheadPoint,
                report.remainingPathDistance)
    }

    fun hasPassedMarker(marker: String): Boolean{
        return mPath.hasPassedMarker(marker)
    }
    fun isFinished(): Boolean {
        return mAtEndOfPath
    }

}