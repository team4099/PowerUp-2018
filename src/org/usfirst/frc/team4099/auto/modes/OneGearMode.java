package org.usfirst.frc.team4099.auto.modes;

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.auto.actions.ForwardAction;
import org.usfirst.frc.team4099.auto.actions.SetIntakeAction;
import org.usfirst.frc.team4099.auto.actions.TurnAction;
import org.usfirst.frc.team4099.auto.actions.WaitAction;
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters;
import org.usfirst.frc.team4099.lib.util.LiftVision;
import org.usfirst.frc.team4099.lib.util.Rotation2D;
import org.usfirst.frc.team4099.lib.util.Utils;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.subsystems.Intake;

import java.io.FileNotFoundException;

/**
 * Created by plato2000 on 2/13/17.
 */
public class OneGearMode extends AutoModeBase {

    private final double initialForwardDistance;
    private final Rotation2D initialTurn;
    private final boolean backOut;
    private final boolean turnAround;


    public OneGearMode(AutonomousInitParameters initParameters, boolean backOut, boolean turnAround) {
        this.backOut = backOut;
        this.turnAround = turnAround;
        this.initialForwardDistance = initParameters.getDistanceInMeters();
        this.initialTurn = initParameters.getTurnAngle();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ForwardAction(initialForwardDistance));
        // If in center lane (already lined up with peg - don't use vision here
        // TODO: check if using vision is necessary in center lane
        if(initialTurn.getDegrees() == 0) {
            runAction(new SetIntakeAction(Intake.IntakePosition.UP_AND_OPEN));
            runAction(new WaitAction(1));

            if(backOut) {
                runAction(new ForwardAction(Constants.Autonomous.BACK_OUT_INCHES));

                Rotation2D turn = Rotation2D.fromDegrees(90);
                // Go to left side of airship if we're red, otherwise go to right.
                // This is because the loading station is on the right if red, left if blue
                // so if you want to get gears dropped by teammates, going around airship makes it faster
                if(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
                    turn = turn.inverse();
                }
                runAction(new TurnAction(turn));
                runAction(new ForwardAction(Constants.Autonomous.AIRSHIP_WIDTH_INCHES));
                runAction(new TurnAction(turn.inverse()));
                runAction(new ForwardAction(Constants.Autonomous.DISTANCE_PAST_AIRSHIP_INCHES));
            }
        } else {
            runAction(new TurnAction(initialTurn)); // turn towards air ship
            System.out.println("Finished initial turn");
            try {
                LiftVision liftVision = Utils.getLiftLocation();
                System.out.println("Vision Turn: " + liftVision.getTurnAngle());
                System.out.println("Vision distance: " + liftVision.getDistance());
                runAction(new TurnAction(liftVision.getTurnAngle()));
                System.out.println("Finish vision turn");
                runAction(new ForwardAction(liftVision.getDistance()));
                runAction(new SetIntakeAction(Intake.IntakePosition.UP_AND_OPEN));
                runAction(new WaitAction(Constants.Autonomous.WAIT_TIME_ON_LIFT));
                if(backOut) {
                    runAction(new ForwardAction(Constants.Autonomous.BACK_OUT_INCHES));
                    runAction(new TurnAction(liftVision.getTurnAngle().inverse()));
                }

            } catch (FileNotFoundException e) {
                System.out.println("Couldn't find lift... fix ur vision");
                runAction(new ForwardAction(20));
            }
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
