package org.usfirst.frc.team4099.auto.modes;

import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.auto.actions.ForwardAction;
import org.usfirst.frc.team4099.auto.actions.SetIntakeAction;
import org.usfirst.frc.team4099.auto.actions.TurnAction;
import org.usfirst.frc.team4099.auto.actions.WaitAction;
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters;
import org.usfirst.frc.team4099.lib.util.Rotation2D;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.subsystems.Intake;

/**
 * Created by plato2000 on 2/13/17.
 */
public class OneGearMode extends AutoModeBase {

    private final double initialForwardSeconds;
    private final Rotation2D initialTurn;
    private final boolean backOut;
    private final boolean turnAround;


    public OneGearMode(AutonomousInitParameters initParameters, boolean backOut, boolean turnAround) {
        this.backOut = backOut;
        this.turnAround = turnAround;
        this.initialForwardSeconds = initParameters.getInitalForwardSeconds();
        this.initialTurn = initParameters.getTurnAngle();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // If in center lane (already lined up with peg - don't use vision here
        // TODO: check if using vision is necessary in center lane
        System.out.println("initialTurn " + initialTurn.getDegrees());
        if(initialTurn.getDegrees() == 0) {
            runAction(new ForwardAction(initialForwardSeconds, true, true));
            runAction(new SetIntakeAction(Intake.IntakePosition.DOWN_AND_OPEN));
            runAction(new WaitAction(.75));
            runAction(new ForwardAction(-1));

        } else {
            runAction(new ForwardAction(initialForwardSeconds, false, true));
            runAction(new TurnAction(initialTurn)); // turn towards air ship
            System.out.println("Finished initial turn");
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
            System.out.println("Couldn't find lift... fix ur vision");
            runAction(new ForwardAction(4, true, false));
            runAction(new SetIntakeAction(Intake.IntakePosition.DOWN_AND_OPEN));
            runAction(new ForwardAction(-1, false, false));
//            }
            if(backOut){
                // back out and go past airship
                runAction(new TurnAction(initialTurn.inverse()));
                runAction(new ForwardAction(Constants.Autonomous.DISTANCE_PAST_AIRSHIP_INCHES));
            }
        }
        if(turnAround) {
            runAction(new TurnAction(Rotation2D.BACKWARDS, true));
        }

    }
}
