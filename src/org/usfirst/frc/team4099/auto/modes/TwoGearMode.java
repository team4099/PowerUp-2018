package org.usfirst.frc.team4099.auto.modes;

//one gear mode, back out, then turn towards driver station and connect to "get_gear", then implement the rest

import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.auto.actions.ForwardAction;
import org.usfirst.frc.team4099.auto.actions.SetIntakeAction;
import org.usfirst.frc.team4099.auto.actions.TurnAction;
import org.usfirst.frc.team4099.auto.actions.WaitAction;
import org.usfirst.frc.team4099.lib.util.*;
import org.usfirst.frc.team4099.robot.Constants;
import org.usfirst.frc.team4099.robot.subsystems.Intake;

import java.io.FileNotFoundException;

/**
 * Created by plato2000 on 2/13/17.
 */
public class TwoGearMode extends OneGearMode {
    private static final double BACK_OUT_AMOUNT = -40;
    private final Rotation2D initialTurn;
    private final boolean backOut;
    private final boolean turnAround;
    private final double initialForward;

    public TwoGearMode(AutonomousInitParameters initParameters, boolean backOut, boolean turnAround) {
        super(initParameters, false, false);
        this.backOut = backOut;
        this.initialTurn = initParameters.getTurnAngle();
        this.turnAround = turnAround;
        this.initialForward = initParameters.getInitalForwardSeconds();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        super.routine();
        runAction(new ForwardAction(BACK_OUT_AMOUNT));
        runAction(new TurnAction(Rotation2D.BACKWARDS, true));
        try{
            GearVision gearVision = Utils.getGearLocation();
            runAction(new TurnAction(gearVision.getTurnAngle()));
            runAction(new SetIntakeAction(Intake.IntakePosition.DOWN_AND_OPEN));
            runAction(new ForwardAction(gearVision.getDistance() + Constants.Autonomous.EXTRA_INCHES));
            runAction(new SetIntakeAction(Intake.IntakePosition.UP_AND_CLOSED));
            runAction(new WaitAction(Constants.Autonomous.WAIT_TIME_ON_LIFT));
            runAction(new TurnAction(Rotation2D.FORWARDS, true));

            LiftVision liftVision = Utils.getLiftLocation();
            if(Math.abs(liftVision.getOffsetAngle().getDegrees() - liftVision.getTurnAngle().getDegrees()) > 20){
                runAction(new ForwardAction(initialForward));
                runAction(new TurnAction(initialTurn));
                liftVision = Utils.getLiftLocation();
            }
            runAction(new TurnAction(liftVision.getTurnAngle()));
            runAction(new ForwardAction(liftVision.getDistance()));

            runAction(new SetIntakeAction(Intake.IntakePosition.UP_AND_OPEN));
            runAction(new WaitAction(Constants.Autonomous.WAIT_TIME_ON_LIFT));
            if(backOut) {
                runAction(new ForwardAction(BACK_OUT_AMOUNT));
            }

        } catch (FileNotFoundException e){
            System.out.println("Either could not find gear or lift afterward :/");
        }

        if(turnAround) {
            runAction(new TurnAction(Rotation2D.BACKWARDS, true));
        }

    }
}
