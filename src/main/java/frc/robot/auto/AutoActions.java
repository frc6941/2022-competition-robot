package frc.robot.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.coordinators.Superstructure;
import frc.robot.coordinators.Superstructure.STATE;

public class AutoActions {
    public static final HashMap<String, Command> commandMapping = new HashMap<>();

    private static final Command intake = new InstantCommand(() -> Superstructure.getInstance().setWantIntake(true));
    private static final Command stopIntake = new InstantCommand(() -> Superstructure.getInstance().setWantIntake(false));

    private static final Command spit = new InstantCommand(() -> Superstructure.getInstance().setWantSpit(true));
    private static final Command stopSpit = new InstantCommand(() -> Superstructure.getInstance().setWantSpit(false));

    private static final Command fire = new SequentialCommandGroup(
        new WaitUntilCommand(() -> Superstructure.getInstance().isOnTarget()),
        new InstantCommand(() -> Superstructure.getInstance().setState(STATE.SHOOTING)),
        new WaitCommand(1.2),
        new InstantCommand(() -> Superstructure.getInstance().setState(STATE.CHASING))
    );

    private static final Command ejectEnable = new InstantCommand(() -> Superstructure.getInstance().setWantEject(true));
    private static final Command ejectDisable = new InstantCommand(() -> Superstructure.getInstance().setWantEject(false));

    private static final Command intakeAndStop = new SequentialCommandGroup(
        new InstantCommand(() -> Superstructure.getInstance().setWantIntake(true)),
        new WaitCommand(2.0),
        new InstantCommand(() -> Superstructure.getInstance().setWantIntake(false)),
        new InstantCommand(() -> Superstructure.getInstance().setWantSpit(false))
    );
    private static final Command spitAndStop = new SequentialCommandGroup(
        new InstantCommand(() -> Superstructure.getInstance().setWantSpit(true)),
        new WaitCommand(2.0),
        new InstantCommand(() -> Superstructure.getInstance().setWantIntake(false)),
        new InstantCommand(() -> Superstructure.getInstance().setWantSpit(false))
    );

    static {
        commandMapping.put("intake", intake);
        commandMapping.put("stop intake", stopIntake);
        commandMapping.put("spit", spit);
        commandMapping.put("stop spit", stopSpit);
        commandMapping.put("fire", fire);
        commandMapping.put("eject enable", ejectEnable);
        commandMapping.put("eject fisable", ejectDisable);
        commandMapping.put("intake and stop", intakeAndStop);
        commandMapping.put("spit and stop", spitAndStop);
    }
}
