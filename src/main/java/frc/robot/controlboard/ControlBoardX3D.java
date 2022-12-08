package frc.robot.controlboard;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.controlboard.LogitechX3D.Axis;
import frc.robot.controlboard.LogitechX3D.Button;
import frc.robot.controlboard.SwerveCardinal.SWERVE_CARDINAL;

public class ControlBoardX3D {
    public final double kSwerveDeadband = Constants.CONTROLLER_DEADBAND;

    private final int kDpadUp = 0;
    private final int kDpadRight = 90;
    private final int kDpadDown = 180;
    private final int kDpadLeft = 270;

    private static ControlBoardX3D instance = null;

    public static ControlBoardX3D getInstance() {
        if (instance == null) {
            instance = new ControlBoardX3D();
        }
        return instance;
    }

    private final LogitechX3D driver;
    private final CustomXboxController operator;

    private ControlBoardX3D() {
        driver = new LogitechX3D(Constants.DRIVER_CONTROLLER_PORT);
        operator = new CustomXboxController(Constants.OPERATOR_CONTROLLER_PORT);
    }

    public LogitechX3D getDriverController() {
        return driver;
    }

    public CustomXboxController getOperatorController() {
        return operator;
    }

    public void setDriverRumble(double power, double interval) {
        driver.setRumble(power, interval);
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getAxis(Axis.Y);
        double strafeAxis = driver.getAxis(Axis.X);

        forwardAxis = Constants.CONTROLLER_INVERT_Y ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.CONTROLLER_INVERT_X ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            return tAxes.times(Constants.DRIVE_MAX_VELOCITY);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Axis.Z) * 2.0;
        rotAxis = Constants.CONTROLLER_INVERT_R ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < kSwerveDeadband) {
            return 0.0;
        } else {
            return (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
        }
    }

    public boolean zeroGyro() {
        return driver.getController().getRawButtonPressed(Button.SEVEN.id);
    }

    public SWERVE_CARDINAL getSwerveSnapRotation() {
        if (driver.getController().getPOV() == kDpadDown) {
            return SWERVE_CARDINAL.BACKWARDS;
        } else if (driver.getController().getPOV() == kDpadRight) {
            return SWERVE_CARDINAL.RIGHT;
        } else if (driver.getController().getPOV() == kDpadLeft) {
            return SWERVE_CARDINAL.LEFT;
        } else if (driver.getController().getPOV() == kDpadUp) {
            return SWERVE_CARDINAL.FORWARDS;
        } else {
            return SWERVE_CARDINAL.NONE;
        }
    }

    // Locks wheels in X formation
    public boolean getSwerveBrake() {
        return driver.getButton(Button.FOUR);
    }

    // Robot oriented switch
    public boolean getSwitchRobotOrientedDrive() {
        return driver.getButton(Button.FIVE);
    }

    public boolean getSwitchCompressorForceEnable() {
        return driver.getButton(Button.TWELVE);
    }

    // Start intake
    public boolean getIntake() {
        return driver.getButton(Button.SIDE);
    }

    public boolean getShoot() {
        return driver.getButton(Button.TRIGGER);
    }

    public boolean getSpit() {
        return driver.getController().getPOV() == kDpadDown;
    }

    // Climber Controls
    public boolean getEnterClimbMode() {
        return operator.getButton(CustomXboxController.Button.LB) && operator.getButton(CustomXboxController.Button.RB)
                && operator.getTriggerBoolean(CustomXboxController.Side.LEFT)
                && operator.getTriggerBoolean(CustomXboxController.Side.RIGHT);
        // return operator.getButton(Button.LB) && operator.getButton(Button.RB);
    }

    public boolean getExitClimbMode() {
        return operator.getButton(CustomXboxController.Button.BACK)
                && operator.getButton(CustomXboxController.Button.START);
    }

    public boolean getToggleOpenLoopClimbMode() {
        return operator.getController().getLeftStickButtonPressed();
    }

    public boolean getClimberRetract() {
        return operator.getController().getPOV() == kDpadDown;
    }

    public boolean getClimberExtend() {
        return operator.getController().getPOV() == kDpadUp;
    }

    public boolean getClimberDown() {
        return operator.getController().getPOV() == kDpadDown;
    }

    public boolean getClimberUp() {
        return operator.getController().getPOV() == kDpadUp;
    }

    public boolean getClimberHootOut() {
        return operator.getController().getPOV() == kDpadRight;
    }

    public boolean getClimberHootIn() {
        return operator.getController().getPOV() == kDpadLeft;
    }

    public boolean getTraversalClimb() {
        return operator.getController().getYButtonPressed();
    }

    public boolean getHighBarClimb() {
        return operator.getController().getBButtonPressed();
    }

    public boolean getClimbAutoConfirmation() {
        return operator.getButton(CustomXboxController.Button.A);
    }

    public boolean getClimbAutoAbort() {
        return operator.getButton(CustomXboxController.Button.X);
    }
}
