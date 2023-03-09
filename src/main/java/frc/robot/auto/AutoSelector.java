package frc.robot.auto;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.auto.basics.AutoActions;
import frc.robot.commands.AutoScore;
import frc.robot.states.ScoringTarget;
import frc.robot.states.ScoringTarget.SCORING_GRID;
import frc.robot.states.ScoringTarget.SCORING_ROW;
import frc.robot.states.ScoringTarget.SCORING_SIDE;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.SJTUSwerveMK5Drivebase;
import frc.robot.subsystems.TargetSelector;

public class AutoSelector {
    public enum AUTO_START_POSITION {
        INNER,
        OUTER,
        CENTER
    }

    public enum AUTO_ACTION {
        DO_NOTHING,
        MOBILITY,
        SCORE_PRELOAD,
        TWO_GAMEPIECE,
        LINK
    }

    public enum AUTO_BALANCE {
        YES,
        NO
    }

    private final SendableChooser<AUTO_START_POSITION> autoStartPosition = new SendableChooser<>();
    private final SendableChooser<AUTO_ACTION> autoAction = new SendableChooser<>();
    private final SendableChooser<AUTO_BALANCE> autoBalance = new SendableChooser<>();
    private boolean autoWarning = false;

    private final AutoActions autoBuilder;
    private Command builtAutoCommand;

    private AutoSelector() {
        autoStartPosition.addOption("Inner (Field Side)", AUTO_START_POSITION.INNER);
        autoStartPosition.addOption("Outer (Wall Side)", AUTO_START_POSITION.OUTER);
        autoStartPosition.addOption("Middle", AUTO_START_POSITION.CENTER);

        autoAction.setDefaultOption("Do Nothing", AUTO_ACTION.DO_NOTHING);
        autoAction.addOption("Mobility", AUTO_ACTION.DO_NOTHING);
        autoAction.addOption("Score Preload", AUTO_ACTION.SCORE_PRELOAD);
        autoAction.addOption("Score Preload, then Get Cube", AUTO_ACTION.TWO_GAMEPIECE);
        autoAction.addOption("Score Link", AUTO_ACTION.LINK);

        autoBalance.setDefaultOption("DO NOT Balance", AUTO_BALANCE.NO);
        autoBalance.setDefaultOption("Balance - MAKE SURE THAT YOUR TEAMMATES DO NOT", AUTO_BALANCE.YES);


        autoBuilder = new AutoActions(SJTUSwerveMK5Drivebase.getInstance(), ArmAndExtender.getInstance(), Intaker.getInstance(), TargetSelector.getInstance());
    }

    public static AutoSelector getInstance() {
        if (instance == null) {
            instance = new AutoSelector();
        }
        return instance;
    }

    private static AutoSelector instance;

    public void update() {
    }

    public Command getAutoCommand() {
        return builtAutoCommand;
    }

    public boolean getAutoWarning() {
        return autoWarning;
    }
}
