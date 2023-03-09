package frc.robot.subsystems;

import org.frcteam6941.led.AddressableLEDWrapper;
import org.frcteam6941.looper.UpdateManager.Updatable;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.team254.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controlboard.ControlBoard;
import frc.robot.states.AssistedPoseBuilder;
import frc.robot.states.Direction;
import frc.robot.states.GamePiece;
import frc.robot.states.LoadingTarget;
import frc.robot.states.LoadingTarget.LOADING_LOCATION;
import frc.robot.states.ScoringTarget;
import frc.robot.states.ScoringTarget.SCORING_GRID;
import frc.robot.states.ScoringTarget.SCORING_ROW;
import frc.robot.states.ScoringTarget.SCORING_SIDE;
import frc.robot.states.SuperstructureState;
import frc.robot.states.SuperstructureStateBuilder;

public class TargetSelector extends SubsystemBase implements Updatable {
    @AutoLog
    public static class TargetSelectorPeriodicIO {
        // INPUT
        public double[] targetSelect = new double[] { 6, 2 };
    }
    
    public TargetSelectorPeriodicIOAutoLogged mPeriodicIO = new TargetSelectorPeriodicIOAutoLogged();


    private GamePiece targetGamePiece = GamePiece.CONE;
    private ScoringTarget scoringTarget = new ScoringTarget(SCORING_ROW.MID, SCORING_GRID.INNER, SCORING_SIDE.OUTER);
    private LoadingTarget loadingTarget = new LoadingTarget(LOADING_LOCATION.DOUBLE_SUBSTATION_OUTER);

    private Direction scoringDirection = Direction.NEAR;
    private Direction commutingDirection = Direction.NEAR;
    private Direction loadingDirection = Direction.NEAR;

    private ControlBoard mControlBoard = ControlBoard.getInstance();
    
    private AddressableLEDWrapper mIndicator;

    private static TargetSelector instance;

    public static TargetSelector getInstance() {
        if (instance == null) {
            instance = new TargetSelector();
        }
        return instance;
    }
    
    private TargetSelector() {
        mIndicator = new AddressableLEDWrapper(9, 10);
        mIndicator.setIntensity(1.0);
        mIndicator.start(0.05);
    }

    public GamePiece getTargetGamePiece() {
        return this.targetGamePiece;
    }

    public void setTargetGamePiece(GamePiece targetGamePiece) {
        this.targetGamePiece = targetGamePiece;
    }

    public ScoringTarget getScoringTarget() {
        return this.scoringTarget;
    }

    public void setScoringTarget(ScoringTarget scoringTarget) {
        this.scoringTarget = scoringTarget;
    }

    public LoadingTarget getLoadingTarget() {
        return this.loadingTarget;
    }

    public void setLoadingTarget(LoadingTarget loadingTarget) {
        this.loadingTarget = loadingTarget;
    }

    public Direction getScoringDirection() {
        return this.scoringDirection;
    }

    public void setScoringDirection(Direction scoringDirection) {
        this.scoringDirection = scoringDirection;
    }

    public Direction getCommutingDirection() {
        return this.commutingDirection;
    }

    public Direction getLoadingDirection() {
        return this.loadingDirection;
    }

    public SuperstructureState getCommuteSuperstructureState() {
        return SuperstructureStateBuilder.buildCommutingSuperstructureState(commutingDirection);
    }

    public SuperstructureState getScoreSuperstructureState() {
        return SuperstructureStateBuilder.buildScoringSupertructureState(scoringTarget, scoringDirection, targetGamePiece);
    }

    public SuperstructureState getScoreLowerDelSuperstructureState() {
        return SuperstructureStateBuilder.buildScoringSupertructureStateLowerDelta(scoringTarget, scoringDirection, targetGamePiece);
    }
    
    public Pose2d getScorePose2d() {
        return AssistedPoseBuilder.buildScoringPose2d(scoringTarget, scoringDirection, targetGamePiece);
    }

    public SuperstructureState getLoadSuperstructureState() {
        return SuperstructureStateBuilder.buildLoadingSupertructureState(loadingTarget, loadingDirection);
    }

    public SuperstructureState getLoadSuperstructureStateMinExtenderLength() {
        return new SuperstructureState(
            SuperstructureStateBuilder.buildLoadingSupertructureState(loadingTarget, loadingDirection).armAngle,
            Constants.SUBSYSTEM_SUPERSTRUCTURE.CONSTRAINTS.EXTENDER_RANGE.min
        );
    }

    public Pose2d getLoadPose2d() {
        return AssistedPoseBuilder.buildLoadingPose2d(loadingTarget, loadingDirection);
    }

    public SuperstructureState getHairTriggerSuperstructureState() {
        return SuperstructureStateBuilder.buildHairTriggerSuperstructureState();
    }

    public double getIntakerIntakePercentage() {
        switch(targetGamePiece) {
            case CONE:
                return Constants.SUBSYSTEM_INTAKE.INTAKING_PERCENTAGE_CONE;
            case CUBE:
                return Constants.SUBSYSTEM_INTAKE.INTAKING_PERCENTAGE_CUBE;
            default:
                return 0.0;
        }
    }

    public double getIntakerHoldPercentage() {
        switch(targetGamePiece) {
            case CONE:
                return Constants.SUBSYSTEM_INTAKE.HOLD_PERCENTAGE_CONE;
            case CUBE:
                return Constants.SUBSYSTEM_INTAKE.HOLD_PERCENTAGE_CUBE;
            default:
                return 0.0;
        }
    }

    private int[] clampTargetSelect(int[] ids) {
        return new int[] { (int) Util.clamp(ids[0], 0, 8), (int) Util.clamp(ids[1], 0, 2) };
    }

    @Override
    public synchronized void read(double time, double dt){
        double[] temp = mPeriodicIO.targetSelect;
        if(mControlBoard.getTargetMoveDown()) {
            temp[1] -= 1;
        }
        if(mControlBoard.getTargetMoveUp()) {
            temp[1] += 1;
        }
        if(mControlBoard.getTargetMoveRight()) {
            temp[0] -= 1;
        }
        if(mControlBoard.getTargetMoveLeft()) {
            temp[0] += 1;
        }

        int[] transformed = new int[] { 0, 0 };
        for(int i = 0; i < temp.length; i++) {
            transformed[i] = (int) temp[i];
        }
        transformed = clampTargetSelect(transformed);

        double[] record = new double[] { 0, 0 };
        for(int i = 0; i < transformed.length; i++) {
            record[i] = (double) transformed[i];
        }

        if(record.equals(mPeriodicIO.targetSelect)) {

        } else {
            mPeriodicIO.targetSelect = record;
            scoringTarget = new ScoringTarget(transformed);
            if(scoringTarget.getScoringSide() == SCORING_SIDE.MIDDLE) {
                this.targetGamePiece = GamePiece.CUBE;
            } else {
                this.targetGamePiece = GamePiece.CONE;
            }
        }

        if(mControlBoard.getLoadGround()) {
            this.loadingTarget.setLoadingLocation(LOADING_LOCATION.GROUND);
        }
        if(mControlBoard.getLoadStation()) {
            this.loadingTarget.setLoadingLocation(LOADING_LOCATION.DOUBLE_SUBSTATION_INNER);
        }
    }
    
    @Override
    public synchronized void update(double time, double dt){
        switch(targetGamePiece) {
            case CONE:
                if(scoringTarget.getScoringRow() == SCORING_ROW.HIGH) {
                    scoringDirection = Direction.NEAR;
                    commutingDirection = Direction.NEAR;
                    loadingDirection = Direction.NEAR;
                } else {
                    scoringDirection = Direction.FAR;
                    commutingDirection = Direction.FAR;
                    loadingDirection = Direction.FAR;
                }
                break;
            case CUBE:
                scoringDirection = Direction.FAR;
                commutingDirection = Direction.FAR;
                loadingDirection = Direction.FAR;
                break;
        }
    }
    
    @Override
    public synchronized void write(double time, double dt){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void telemetry(){
        Logger.getInstance().processInputs("Target Selector", mPeriodicIO);

    }
    
    @Override
    public synchronized void start(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void stop(){
        // Auto Generated Method
    }
    
    @Override
    public synchronized void simulate(double time, double dt){
        double[] temp = mPeriodicIO.targetSelect;
        if(mControlBoard.getTargetMoveDown()) {
            temp[1] -= 1;
        }
        if(mControlBoard.getTargetMoveUp()) {
            temp[1] += 1;
        }
        if(mControlBoard.getTargetMoveRight()) {
            temp[0] -= 1;
        }
        if(mControlBoard.getTargetMoveLeft()) {
            temp[0] += 1;
        }

        int[] transformed = new int[] { 0, 0 };
        for(int i = 0; i < temp.length; i++) {
            transformed[i] = (int) temp[i];
        }
        transformed = clampTargetSelect(transformed);

        double[] record = new double[] { 0, 0 };
        for(int i = 0; i < transformed.length; i++) {
            record[i] = (double) transformed[i];
        }
        mPeriodicIO.targetSelect = record;

        scoringTarget = new ScoringTarget(transformed);
        if(scoringTarget.getScoringSide() == SCORING_SIDE.MIDDLE) {
            this.targetGamePiece = GamePiece.CUBE;
        } else {
            this.targetGamePiece = GamePiece.CONE;
        }

        if(mControlBoard.getLoadGround()) {
            this.loadingTarget.setLoadingLocation(LOADING_LOCATION.GROUND);
        }
        if(mControlBoard.getLoadStation()) {
            this.loadingTarget.setLoadingLocation(LOADING_LOCATION.DOUBLE_SUBSTATION_INNER);
        }
    }
}
