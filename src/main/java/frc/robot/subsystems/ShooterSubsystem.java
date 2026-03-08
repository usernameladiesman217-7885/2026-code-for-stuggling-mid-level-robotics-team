package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    public SparkMax chain_motor = new SparkMax(12, MotorType.kBrushless);
    public SparkMax top_motor = new SparkMax(13, MotorType.kBrushless);
    public SparkMaxConfig chain_config = new SparkMaxConfig();
    public SparkMaxConfig top_config = new SparkMaxConfig();

    public ShooterSubsystem()
    {
        chain_config.inverted(false);
        top_config.inverted(true);
        chain_config.idleMode(IdleMode.kBrake);
        top_config.idleMode(IdleMode.kBrake);
        chain_motor.configure(chain_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        top_motor.configure(top_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

public void setSpeed (double speed)
{
    chain_motor.set(speed);
    top_motor.set(speed);
}

    
}
