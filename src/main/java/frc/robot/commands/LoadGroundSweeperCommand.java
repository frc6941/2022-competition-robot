package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.motion.SuperstructureKinematics;
import frc.robot.states.Direction;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.ArmAndExtender;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.TargetSelector;

public class LoadGroundSweeperCommand extends SequentialCommandGroup {
    ArmAndExtender mSuperstructure;
    Intaker mIntaker;
    DoubleSupplier extensionDistance;
    TargetSelector mTargetSelector;

    private Supplier<Translation2d> initialPosition;
    private Supplier<Translation2d> finalPosition;
    private Supplier<SuperstructureState> superstructureTargetSupplier;
    private Supplier<SuperstructureState> baseSuperstructureTargetSupplier;
    
    private SlewRateLimiter xLimiter = new SlewRateLimiter(2.0);

    private final double maxExtensionDistance = 1.20;

    public LoadGroundSweeperCommand(
        ArmAndExtender mSuperstructure,
        Intaker mIntaker,
        TargetSelector mTargetSelector,
        DoubleSupplier extensionDistance
    ) {
        this.mSuperstructure = mSuperstructure;
        this.mIntaker = mIntaker;
        this.extensionDistance = extensionDistance;
        this.mTargetSelector = mTargetSelector;

        initSuppliers();
        addCommands(
            new RequestSuperstructureStateAutoRetract(mSuperstructure, baseSuperstructureTargetSupplier)
            .andThen(
                new InstantCommand(() -> mIntaker.runIntake(() -> mTargetSelector.getTargetGamePiece()))
                .andThen(new InstantCommand(() -> xLimiter.reset(extensionDistance.getAsDouble())))
                .andThen(
                    new RequestSuperstructureStateCommand(mSuperstructure, superstructureTargetSupplier).repeatedly()
                    .until(() -> mIntaker.hasGamePiece())
                )
            )
            .andThen(
                new RequestSuperstructureStateAutoRetract(mSuperstructure, () -> mTargetSelector.getCommuteSuperstructureState())
            )
            .unless(() -> mIntaker.hasGamePiece())
        );
    }

    public void initSuppliers() {
        initialPosition = () -> mTargetSelector.getLoadingDirection() == Direction.NEAR
                ? new Translation2d(0.60, 0.05)
                : new Translation2d(-0.60, 0.05);
        finalPosition = () -> mTargetSelector.getLoadingDirection() == Direction.NEAR
                ? new Translation2d(0.60 + maxExtensionDistance, 0.05)
                : new Translation2d(-0.60 - maxExtensionDistance, 0.05);

        this.superstructureTargetSupplier = () -> SuperstructureKinematics.inverseKinematics2d(
                initialPosition.get().interpolate(finalPosition.get(), xLimiter.calculate(extensionDistance.getAsDouble())));
        this.baseSuperstructureTargetSupplier = () -> SuperstructureKinematics.inverseKinematics2d(
                initialPosition.get().interpolate(finalPosition.get(), 0.0));
    }
}
