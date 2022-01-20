package org.frcteam6941.commands.basic;

import java.util.function.DoubleSupplier;

import org.frcteam6941.swerve.SwerveDrivetrainBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class SwerveDriveCommand extends CommandBase {
    private final double pedal = 0.45;
    private final double DEADBAND = Constants.DEADBAND;
    private final boolean INVERT_X = Constants.INVERT_X;
    private final boolean INVERT_Y = Constants.INVERT_Y;
    private final boolean INVERT_R = Constants.INVERT_R;

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;

    private SwerveDrivetrainBase s_Swerve;
    private DoubleSupplier translationAxis;
    private DoubleSupplier strafeAxis;
    private DoubleSupplier rotationAxis;
    private DoubleSupplier scaleAxis;

    /**
     * Driver teleop command constructor.
     * 
     * @param s_Swerve        The swerve drivetrain.
     * @param translationAxis The axis responsible for 'forward'.
     * @param strafeAxis      The axis responsible for 'sides'.
     * @param scaleAxis       The axis that provide scale factor, increasing the
     *                        speed of the bot.
     * @param rotationAxis    The axis that provide rotational speed.
     * @param fieldRelative   Whether the robot is operated in field-oriented mode.
     */
    public SwerveDriveCommand(SwerveDrivetrainBase s_Swerve, DoubleSupplier translationAxis, DoubleSupplier strafeAxis,
            DoubleSupplier scaleAxis, DoubleSupplier rotationAxis, boolean fieldRelative) {

        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.scaleAxis = scaleAxis;
        this.fieldRelative = fieldRelative;
    }

    private double applyRotationalDeadband(double input) {
        double deadband = DEADBAND;
        if (Math.abs(input) < deadband) {
            return 0.0;
        } else {
            return (input - (Math.signum(input) * deadband)) / (1 - deadband);
        }
    }

    private Translation2d applyTranslationalDeadband(Translation2d input) {
        double x;
        double y;
        if (Math.abs(input.getX()) <= DEADBAND) {
            x = 0;
        } else {
            x = input.getX();
        }

        if (Math.abs(input.getY()) <= DEADBAND) {
            y = 0;
        } else {
            y = input.getY();
        }
        return new Translation2d(x, y);
    }

    public double[] getAxes() {
        double yAxis = -translationAxis.getAsDouble();
        double xAxis = -strafeAxis.getAsDouble();
        double rAxis = -rotationAxis.getAsDouble();

        Translation2d tAxes;

        /* Deadbands */
        tAxes = applyTranslationalDeadband(new Translation2d(yAxis, xAxis));
        rAxis = applyRotationalDeadband(rAxis);

        double[] axes = { tAxes.getX(), tAxes.getY(), rAxis };

        return axes;
    }

    @Override
    public void execute() {

        double yAxis;
        double xAxis;
        double rAxis;
        double scaleFactor;

        Translation2d tAxes; // translational axis
        /* Inversions */
        yAxis = INVERT_Y ? translationAxis.getAsDouble() : -translationAxis.getAsDouble();
        xAxis = INVERT_X ? strafeAxis.getAsDouble() : -strafeAxis.getAsDouble();
        rAxis = INVERT_R ? rotationAxis.getAsDouble() : -rotationAxis.getAsDouble();

        scaleFactor = scaleAxis.getAsDouble();

        /* Deadbands */
        tAxes = applyTranslationalDeadband(new Translation2d(yAxis, xAxis));
        rAxis = applyRotationalDeadband(rAxis);

        translation = new Translation2d(tAxes.getX(), tAxes.getY()).times(1 - pedal + pedal * scaleFactor);
        rotation = rAxis * (1 - pedal + pedal * scaleFactor);
        if(Math.abs(rotation) >= 0.01){
            s_Swerve.setLockHeading(false);
        }
        s_Swerve.drive(translation, rotation, fieldRelative);
    }

    public Translation2d getChassisTranslation() {
        double[] axes = getAxes();

        double yAxis = axes[0];
        double xAxis = axes[1];

        translation = new Translation2d(yAxis, xAxis).times(Constants.DRIVE_MAX_VELOCITY);

        return translation;
    }

    public double getChassisRotation() {
        double[] axes = getAxes();

        double rAxis = axes[2];

        rotation = rAxis * Constants.DRIVE_MAX_ANGULAR_VELOCITY;

        return rotation;
    }

}