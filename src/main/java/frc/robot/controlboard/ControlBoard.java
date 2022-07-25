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
            double pedalScale = 1.0 - Constants.CONTROLLER_PEDAL + Constants.CONTROLLER_PEDAL * pedal;
            return tAxes.times(pedalScale);
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

    // Robot oriented switch
    public boolean getRobotOrientedDrive() {
        return driver.getController().getRightStickButton();
    }

    // Start intake
    public boolean getIntake() {
        return driver.getButton(Button.RB);
    }

    public boolean getShoot() {
        return driver.getButton(Button.LB);
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

    public boolean getClimberHootOut(){
        return operator.getController().getPOV() == kDpadRight;
    }

    public boolean getClimberHootIn(){
        return operator.getController().getPOV() == kDpadLeft;
    }

    public boolean getTraversalClimb() {
        return operator.getButton(Button.LB) && operator.getController().getYButtonPressed();
    }
    
    public boolean getHighBarClimb() {
        return operator.getButton(Button.LB) && operator.getController().getBButtonPressed();
    }

}
