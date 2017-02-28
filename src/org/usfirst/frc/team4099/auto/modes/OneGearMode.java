package org.usfirst.frc.team4099.auto.modes;
// put in a gear, and go put it on the peg

import edu.wpi.first.wpilibj.DriverStation;
import org.usfirst.frc.team4099.auto.AutoModeEndedException;
import org.usfirst.frc.team4099.auto.actions.ForwardAction;
import org.usfirst.frc.team4099.auto.actions.SetIntakeAction;
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
        // If in center lane (already lined up with peg - don't use vision here
        // TODO: check if using vision is necessary in center lane
        if(initialTurn.getDegrees() == 0) {
            runAction(new SetIntakeAction(Intake.IntakePosition.UP_AND_OPEN));
            runAction(new WaitAction(0.5));
            runAction(new ForwardAction(BACK_OUT_AMOUNT));

            if(goToBaseline) {
                Rotation2D turn = Rotation2D.fromDegrees(90);
                // Go to left side of airship if we're red, otherwise go to right.
                // This is because the loading station is on the right if red, left if blue
                // so if you want to get gears dropped by teammates, going around airship makes it faster
                if(DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
                    turn = turn.inverse();
                }
                runAction(new TurnAction(turn));
                runAction(new ForwardAction(SIDE_AMOUNT));
                runAction(new TurnAction(turn.inverse()));
                runAction(new ForwardAction(1.7));
            }
        } else {
            runAction(new TurnAction(initialTurn)); // turn towards air ship
            try {
                double[] visionInfo = Utils.getNumbersFromUrl("get_lift"); // vision[1]=turn, vision[2]=dis(m)
                Rotation2D turnToPeg = Rotation2D.fromDegrees(visionInfo[1]);
                double distanceToPeg = visionInfo[2];
                runAction(new TurnAction(turnToPeg));
                runAction(new ForwardAction(distanceToPeg));
                runAction(new SetIntakeAction(Intake.IntakePosition.UP_AND_OPEN));
                runAction(new WaitAction(0.5));
                runAction(new ForwardAction(BACK_OUT_AMOUNT));
                runAction(new TurnAction(turnToPeg.inverse()));

            } catch (FileNotFoundException e) {
                System.out.println("Couldn't find lift... fix ur vision");
            }
            if(goToBaseline){
                // back out and go to baseline
                runAction(new TurnAction(initialTurn.inverse()));
                runAction(new ForwardAction(1.5));
            }
        }
        if(turnAround) {
            runAction(new TurnAction(Rotation2D.fromDegrees(179.9)));
        }

    }
}
