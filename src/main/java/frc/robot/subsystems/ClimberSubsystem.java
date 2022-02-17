package frc.robot.subsystems;

import java.util.Optional;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team254.lib.util.TimeDelayedBoolean;
import org.frcteam2910.common.robot.UpdateManager.Updatable;
import org.frcteam6941.utils.LazyTalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.gamepiece.Cargo;
import frc.robot.subsystems.IntakerSubsystem.STATE;

public class ClimberSubsystem extends SubsystemBase implements Updatable {
    private static ClimberSubsystem instance;
    private TalonFX climberMotor = new TalonFX(Constants.CANID.CLIMBER_MOTOR);

    private ClimberSubsystem() {
        climberMotor.configFactoryDefault();
        climberMotor.setNeutralMode(NeutralMode.Brake);
    }

    private STATE state;

    public static ClimberSubsystem getInstance() {
        if (instance == null) {
            instance = new ClimberSubsystem();
        }
        return instance;
    }

    public void setClimberPercentage(double power) {
        this.climberMotor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void update(double time, double dt) {
    }

    public static enum STATE {

    }

    public STATE getState() {
        return this.state;
    }

    public void setState(STATE state) {
        this.state = state;
    }

}
