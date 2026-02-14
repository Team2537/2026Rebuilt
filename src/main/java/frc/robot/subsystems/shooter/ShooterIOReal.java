package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.units.Units;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;


public class ShooterIOReal implements ShooterIO {

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFX hoodMotor;
    private final TalonFX kickerMotor;

    private final TorqueCurrentFOC kickerTorqueRequest = new TorqueCurrentFOC(0.0);
    private final VoltageOut kickerVoltageRequest = new VoltageOut(0.0);
    private final VelocityDutyCycle leftMotorVelocityRequest = new VelocityDutyCycle(0.0);
    private final VelocityDutyCycle rightMotorVelocityRequest = new VelocityDutyCycle(0.0);
    private final PositionDutyCycle hoodPositionRequest = new PositionDutyCycle(0.0);

    public ShooterIOReal() {
        leftMotor = new TalonFX(LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new TalonFX(RIGHT_MOTOR_ID, MotorType.kBrushless);
        hoodMotor = new TalonFX(HOOD_MOTOR_ID, MotorType.kBrushless);
        kickerMotor = new TalonFX(KICKER_MOTOR_ID, MotorType.kBrushless);



    }
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // left motor
        inputs.shooterLeftPositionRad = Units.Rotations.of(leftMotorEncoder.getPosition()).in(Units.Radians);
        inputs.shooterLeftVelocityRpm = leftMotor.getVelocity();
        inputs.shooterLeftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.shooterLeftSupplyCurrentAmps = leftMotor.getOutputCurrent();
        inputs.shooterLeftTempCelcius = leftMotor.getMotorTemperature();


        // right motor
        inputs.shooterRightPositionRad = Units.Rotations.of(rightMotorEncoder.getPosition()).in(Units.Radians);
        inputs.shooterRightVelocityRpm = rightMotor.getVelocity();
        inputs.shooterRightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
        inputs.shooterRightSupplyCurrentAmps = rightMotor.getOutputCurrent();
        inputs.shooterRightTempCelcius = rightMotor.getMotorTemperature();
        
        // hood motor
        inputs.hoodPositionRad = Units.Rotations.of(hoodEncoder.getPosition()).in(Units.Radians);
        inputs.hoodVelocityRpm = hoodMotor.getVelocity();
        inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();
        inputs.hoodSupplyCurrentAmps = hoodMotor.getOutputCurrent();
        inputs.hoodTempCelcius = hoodMotor.getMotorTemperature();

        // kicker motor
        inputs.kickerPositionRad = Units.Rotations.of(kickerEncoder.getPosition()).in(Units.Radians);
        inputs.kickerVelocityRpm = kickerMotor.getVelocity();
        inputs.kickerAppliedVolts = kickerMotor.getAppliedOutput() * kickerMotor.getBusVoltage();
        inputs.hoodTempCelcius = kickerMotor.getMotorTemperature();
    }

    @Override
    public void setLeftVelocity(double rpm) {
        leftMotor.setControl(leftMotorVelocityRequest.withOutput(rpm));

    }
    
    @Override
    public void setRightVelocity(double rpm) {
        rightMotor.setControl(rightMotorVelocityRequest.withOutput(rpm));

    }

    @Override
    public void setHoodAngle(double angle) {
        hoodMotor.setControl(hoodPositionRequest).withOutput(angle.in(Units.Radians));
       
    }

    @Override
    public void setKickerTorqueCurrent(Current amps) {
        kickerMotor.setControl(kickerTorqueRequest.withOutput(amps.in(Units.Amps)));

    }
    @Override
    public void setKickerVoltage(Voltage volts) {
        kickerMotor.setControl(kickerVoltageRequest.withOutput(volts.in(Units.Volts)));

    }
    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        hoodMotor.stopMotor();
        kickerMotor.stopMotor();
        

    }
}
