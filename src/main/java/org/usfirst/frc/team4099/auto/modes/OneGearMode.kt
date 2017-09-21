package org.usfirst.frc.team4099.auto.modes

import org.usfirst.frc.team4099.auto.AutoModeEndedException
import org.usfirst.frc.team4099.auto.actions.ForwardAction
import org.usfirst.frc.team4099.auto.actions.SetIntakeAction
import org.usfirst.frc.team4099.auto.actions.TurnAction
import org.usfirst.frc.team4099.auto.actions.WaitAction
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters
import org.usfirst.frc.team4099.lib.util.Rotation2D
import org.usfirst.frc.team4099.robot.Constants
import org.usfirst.frc.team4099.robot.subsystems.Intake

/**
 * Created by plato2000 on 2/13/17.
 */
open class OneGearMode(initParameters: AutonomousInitParameters, private val backOut: Boolean, private val turnAround: Boolean) : AutoModeBase() {

    private val initialForwardSeconds: Double = initParameters.initalForwardSeconds
    private val initialTurn: Rotation2D = initParameters.turnAngle


    @Throws(AutoModeEndedException::class)
    override fun routine() {
        // If in center lane (already lined up with peg - don't use vision here
        // TODO: check if using vision is necessary in center lane
        println("initialTurn " + initialTurn.degrees)
        if (initialTurn.degrees == 0.0) {
            runAction(ForwardAction(initialForwardSeconds, true, true))
            runAction(SetIntakeAction(Intake.IntakePosition.DOWN_AND_OPEN))
            runAction(WaitAction(.75))
            runAction(ForwardAction(-1.0))

        } else {
            runAction(ForwardAction(initialForwardSeconds, false, true))
            runAction(TurnAction(initialTurn)) // turn towards air ship
            println("Finished initial turn")
            //            try {
            //                LiftVision liftVision = Utils.getLiftLocation();
            //                System.out.println("Vision Turn: " + liftVision.getTurnAngle());
            //                System.out.println("Vision distance: " + liftVision.getDistance());
            //                runAction(new TurnAction(liftVision.getTurnAngle()));
            //                System.out.println("Finish vision turn");
            //                runAction(new SetIntakeAction(Intake.IntakePosition.DOWN_AND_OPEN));
            //                runAction(new ForwardAction(-2));
            //                if(backOut) {
            //                    runAction(new ForwardAction(Constants.Autonomous.BACK_OUT_INCHES));
            //                    runAction(new TurnAction(liftVision.getTurnAngle().inverse()));
            //                }
            //
            //            } catch (FileNotFoundException e) {
            println("Couldn't find lift... fix ur vision")
            runAction(ForwardAction(4.0, true, false))
            runAction(SetIntakeAction(Intake.IntakePosition.DOWN_AND_OPEN))
            runAction(ForwardAction(-1.0, false, false))
            //            }
            if (backOut) {
                // back out and go past airship
                runAction(TurnAction(initialTurn.inverse()))
                runAction(ForwardAction(Constants.Autonomous.DISTANCE_PAST_AIRSHIP_INCHES))
            }
        }
        if (turnAround) {
            runAction(TurnAction(Rotation2D.BACKWARDS, true))
        }

    }
}
