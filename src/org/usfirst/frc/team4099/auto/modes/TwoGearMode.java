package org.usfirst.frc.team4099.auto.modes;

//one gear mode, back out, then turn towards driver station and connect to "get_gear", then implement the rest

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.auto.actions.ForwardAction;
import org.usfirst.frc.team4099.auto.actions.SetGrabberAction;
import org.usfirst.frc.team4099.auto.actions.TurnAction;
import org.usfirst.frc.team4099.auto.actions.WaitAction;
import org.usfirst.frc.team4099.lib.util.AutonomousInitParameters;
import org.usfirst.frc.team4099.lib.util.Rotation2D;
import org.usfirst.frc.team4099.lib.util.Utils;
import org.usfirst.frc.team4099.robot.subsystems.Intake;

import java.io.FileNotFoundException;

/**
 * Created by plato2000 on 2/13/17.
 */
public class TwoGearMode extends OneGearMode {
    private static final double BACK_OUT_AMOUNT = -1.5;
    private final double initialForwardDistance;
    private final Rotation2D initialTurn;
    private final boolean goToBaseline;

    public TwoGearMode(AutonomousInitParameters initParameters, boolean goToBaseline) {
        super(initParameters, false, true); // turnAround always true
        this.goToBaseline = goToBaseline;
        this.initialForwardDistance = initParameters.getDistanceInMeters();
        this.initialTurn = initParameters.getTurnAngle();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.routine();
        try{
            double[] gearVision = Utils.getNumbersFromUrl("get_gear"); // [0] = turn [1] = distance
            Rotation2D turnToGear = Rotation2D.fromDegrees(gearVision[0]);
            double distanceToGear = gearVision[1];
            runAction(new TurnAction(turnToGear));
            runAction(new ForwardAction(distanceToGear));
            runAction(new SetGrabberAction(Intake.GrabberPosition.CLOSED));
            runAction(new TurnAction(Rotation2D.fromDegrees(179.9)));
            runAction(new TurnAction(turnToGear.inverse()));
            runAction(new ForwardAction(distanceToGear));
            // back at spot after OneGearMode, not facing peg yet

            runAction(new TurnAction(turnToGear)); // now facing peg again
            runAction(new ForwardAction(-BACK_OUT_AMOUNT));
            runAction(new SetGrabberAction(Intake.GrabberPosition.OPEN));
            runAction(new WaitAction(1));
            runAction(new ForwardAction(BACK_OUT_AMOUNT));
            // We're backed out of 2nd peg run. Go to baseline like OneGear handles it

            if (initialTurn.getDegrees() == 0){
                // we're in center lane
                Rotation2D turn = Rotation2D.fromDegrees(90);
                if(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
                    turn = turn.inverse();
                }
                runAction(new TurnAction(turn));
                runAction(new ForwardAction(1.37));
                runAction(new TurnAction(turn.inverse()));
                runAction(new ForwardAction(1.7));
            } else{
                // not in center lane
                runAction(new TurnAction(initialTurn.inverse()));
                runAction(new ForwardAction(1));
            }



        }catch(FileNotFoundException e){

        }

    }
}
