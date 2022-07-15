package frc.robot.controlboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.controlboard.CustomXboxController.Axis;
import frc.robot.controlboard.CustomXboxController.Button;
import frc.robot.controlboard.CustomXboxController.Side;

public class ControlBoard {
    private final double kSwerveDeadband = Constants.CONTROLLER_DEADBAND;

    private final int kDpadUp = 0;
    private final int kDpadRight = 90;
    private final int kDpadDown = 180;
    private final int kDpadLeft = 270;

    private static ControlBoard instance = null;

    public enum SwerveCardinal {
        NONE(0),

        FORWARDS(0),
        LEFT(270),
        RIGHT(90),
        BACKWARDS(180);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

    public static ControlBoard getInstance() {
        if (instance == null) {
            instance = new ControlBoard();
        }
        return instance;
    }

    private final CustomXboxController driver;
    private final CustomXboxController operator;

    private ControlBoard() {
        driver = new CustomXboxController(Constants.DRIVER_CONTROLLER_PORT);
        operator = new CustomXboxController(Constants.OPERATOR_CONTROLLER_PORT);
    }
    
    public void setDriverRumble(boolean on) {
        driver.setRumble(on);
    }

    
    public void setOperatorRumble(boolean on) {
        operator.setRumble(on);
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getAxis(Side.LEFT, Axis.Y);
        double strafeAxis = driver.getAxis(Side.LEFT, Axis.X);
        double pedal = driver.getTrigger(Side.RIGHT);

        forwardAxis = Constants.CONTROLLER_INVERT_Y ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.CONTROLLER_INVERT_X ? strafeAxis :-strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadbandDirection = new Rotation2d(tAxes.getX(), tAxes.getY());
            Translation2d deadbandVector = new Translation2d(deadbandDirection.getCos() * kSwerveDeadband, deadbandDirection.getSin() * kSwerveDeadband);

            double scaledX = tAxes.getX() - (deadbandVector.getX()) / (1 - deadbandVector.getX());
            double scaledY = tAxes.getY() - (deadbandVector.getY()) / (1 - deadbandVector.getY());
            double pedalScale = 1.0 - Constants.CONTROLLER_PEDAL + Constants.CONTROLLER_PEDAL * pedal;
            return new Translation2d(scaledX, scaledY).times(pedalScale);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Side.RIGHT, Axis.X);
        rotAxis = Constants.CONTROLLER_INVERT_R ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < kSwerveDeadband) {
            return 0.0;
        } else {
            return (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
        }
    }

    public boolean zeroGyro() {
        return driver.getButton(Button.START);
    }

    public SwerveCardinal getSwerveSnapRotation() {
        switch (driver.getController().getPOV()) {
            case kDpadUp:
                return SwerveCardinal.FORWARDS;
            case kDpadLeft:
                return SwerveCardinal.RIGHT;
            case kDpadRight:
                return SwerveCardinal.LEFT;
            case kDpadDown:
                return SwerveCardinal.BACKWARDS;
            default:
                return SwerveCardinal.NONE;
        }
            
    }

    // Locks wheels in X formation
    public boolean getSwerveBrake() {
        return driver.getButton(Button.L_JOYSTICK);
    }

    // Start intake
    public boolean getIntake() {
        return driver.getButton(Button.RB);
    }

    public boolean getShoot() {
        return driver.getButton(Button.X);
    }


    public double getGuessedAimTarget(){
        return new Rotation2d(-operator.getAxis(Side.LEFT, Axis.X), operator.getAxis(Side.LEFT, Axis.Y)).getDegrees() + 90.0;
    }

    public boolean getManualEject() {
        return operator.getButton(Button.LB);
    }

    // Climber Controls
    public boolean getEnterClimbMode() {
        return operator.getButton(Button.LB) && operator.getButton(Button.RB) && operator.getTriggerBoolean(Side.LEFT) && operator.getTriggerBoolean(Side.RIGHT);
    }

    public boolean getExitClimbMode() {
        return operator.getButton(Button.BACK) && operator.getButton(Button.START);
    }

}
