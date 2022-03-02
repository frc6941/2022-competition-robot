package frc.robot.commands.climb;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberGrabBarCommand extends CommandBase {
    SJTUSwerveMK5Drivebase mDrivebase = SJTUSwerveMK5Drivebase.getInstance();
    ClimberSubsystem mClimberSubsystem = ClimberSubsystem.getInstance();

    double minAngle;
    double maxAngle;
    double stagingGoal;
    double endGoal;

    /**
     * Construct a two-stage grab bar command.
     * 
     * @param stagingGoal First stage goal when angle is not optimal. Need to be
     *                    greater than the extension minimum.
     * @param endGoal     Second stage goal activated when it is free to the top.
     * @param minAngle    Mininimum pitch angle free to the top.
     * @param maxAngle    Maximum pitch angle free to the top.
     */
    public ClimberGrabBarCommand(double endGoal, double minAngle, double maxAngle) {
        addRequirements(mClimberSubsystem);
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.endGoal = endGoal;

        if (this.endGoal < Constants.CLIMBER_SAFE_EXTENSION_MINIMUM) {
            this.endGoal = Constants.CLIMBER_SAFE_EXTENSION_MINIMUM;
        }
    }

    public boolean freeToTheTop() {
        return mDrivebase.getPitch().getDegrees() > minAngle && mDrivebase.getPitch().getDegrees() < maxAngle;
    }

    @Override
    public void initialize() {
        this.mClimberSubsystem.lockClimberHeight(stagingGoal, false);
        this.mClimberSubsystem.tryToExtend();
    }

    @Override
    public void execute() {
        if (this.freeToTheTop()) {
            this.mClimberSubsystem.lockClimberHeight(endGoal, false);
        }
    }

    @Override
    public void end(boolean iterrupted) {
        this.mClimberSubsystem.retractClimber();
    }

    @Override
    public boolean isFinished() {
        return this.mClimberSubsystem.isClimberOnTarget() && Math.abs(this.mClimberSubsystem.getClimberHeight() - endGoal) < Constants.CLIMBER_ON_TARGET_TOLERANCE;
    }

}
