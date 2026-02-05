package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    // Motor ports - configure these as needed (CAN IDs)
    private static final int MOTOR_1_PORT = 22;
    private static final int MOTOR_2_PORT = 23;
    
    // Configurable hardcoded speeds (in percent output, -1.0 to 1.0)
    private static final double MOTOR_1_SPEED = -0.45; // 50% forward
    private static final double MOTOR_2_SPEED = 0.45; // 50% forward
    
    private final SparkFlex motor1;
    private final SparkFlex motor2;
    
    private boolean isRunning = false;
    
    public Shooter() {
        // Initialize motors with CAN IDs and MotorType for Neo Vortex
        motor1 = new SparkFlex(MOTOR_1_PORT, MotorType.kBrushless);
        motor2 = new SparkFlex(MOTOR_2_PORT, MotorType.kBrushless);
        
        // Configure motors for Neo Vortex
        configureMotor(motor1);
        configureMotor(motor2);
    }
    
    private void configureMotor(SparkFlex motor) {
        SparkFlexConfig config = new SparkFlexConfig();
        // Configure for Neo Vortex motor - coast mode for shooter
        config.idleMode(IdleMode.kCoast);
        // Note: ResetMode and PersistMode are deprecated but still required by API
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void start() {
        isRunning = true;
        applyOutputs();
    }
    
    public void stop() {
        isRunning = false;
        applyOutputs();
    }
    
    public boolean isRunning() {
        return isRunning;
    }
    
    @Override
    public void periodic() {
        // Re-apply outputs to ensure the controller stays commanded
        if (!DriverStation.isEnabled()) {
            if (isRunning) {
                stop();
            }
            return;
        }
        if (isRunning) {
            applyOutputs();
        }
    }

    private void applyOutputs() {
        double speed1 = isRunning ? MOTOR_1_SPEED : 0.0;
        double speed2 = isRunning ? MOTOR_2_SPEED : 0.0;
        motor1.set(speed1);
        motor2.set(speed2);
    }
}
