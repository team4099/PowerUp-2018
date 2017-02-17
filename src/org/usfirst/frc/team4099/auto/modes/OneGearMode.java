package org.usfirst.frc.team4099.auto.modes;

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
public class OneGearMode extends AutoModeBase {

    private static final double BACK_OUT_AMOUNT = -1.5;
    private static final double SIDE_AMOUNT = 3;
    private static final double BASELINE_DISTANCE = 2.6;

    private final double initialForwardDistance;
    private final Rotation2D initialTurn;
    private final boolean goToBaseline;
    private final boolean turnAround;

    public OneGearMode(AutonomousInitParameters initParameters, boolean goToBaseline, boolean turnAround) {
        this.goToBaseline = goToBaseline;
        this.turnAround = turnAround;
        this.initialForwardDistance = initParameters.getDistanceInMeters();
        this.initialTurn = initParameters.getTurnAngle();
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ForwardAction(initialForwardDistance));
        if(initialTurn.getDegrees() == 0) {
            runAction(new SetGrabberAction(Intake.GrabberPosition.OPEN));
            runAction(new WaitAction(1));
            runAction(new ForwardAction(BACK_OUT_AMOUNT));
            if(goToBaseline) {
                Rotation2D turn = Rotation2D.fromDegrees(90);
                if(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
                    turn = turn.inverse();
                }
                runAction(new TurnAction(turn));
                runAction(new ForwardAction(SIDE_AMOUNT));
                runAction(new TurnAction(turn.inverse()));
                runAction(new ForwardAction(BASELINE_DISTANCE - (initialForwardDistance - BACK_OUT_AMOUNT)));
            }
        }
        runAction(new TurnAction(initialTurn));
        try {
            double[] visionInfo = Utils.getNumbersFromUrl("get_lift");
        } catch (FileNotFoundException e) {
            runAction(new TurnAction(initialTurn.inverse()));
        }
        if(turnAround) {
            runAction(new TurnAction(Rotation2D.fromDegrees(179.9)));
        }
    }
}
