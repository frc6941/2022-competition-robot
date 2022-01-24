import org.frcteam6941.utils.AngleNormalization;

import frc.robot.Constants;

public class LauncherCoordinatorTest {
    public static void main(String[] args){
        double drivetrainHeadingSimulation = 270.0;
        double turretAngleSimulation = 15.0;
        double desiredAngle = 200.0;
        boolean isLimited = true;
        
        double currentDrivetrainAngle = AngleNormalization.getAbsoluteAngleDegree(drivetrainHeadingSimulation);
        double currentTurretAngle = currentDrivetrainAngle + turretAngleSimulation;
        double delta = AngleNormalization.placeInAppropriate0To360Scope(currentTurretAngle, desiredAngle) - currentTurretAngle;
        double availableTurretDelta;
        if(isLimited){
            availableTurretDelta = Math.copySign(Constants.TURRET_SAFE_ZONE_DEGREE, delta) - turretAngleSimulation;
        } else{
            availableTurretDelta = Math.copySign(Constants.TURRET_MAX_ROTATION_DEGREE, delta) - turretAngleSimulation;
        }

        
        if(Math.abs(delta) <= Math.abs(availableTurretDelta)){
            System.out.println(delta + turretAngleSimulation);
        } else{
            System.out.println((availableTurretDelta + turretAngleSimulation) + "    " + (delta - availableTurretDelta + currentDrivetrainAngle));
        }
        
    }
}
