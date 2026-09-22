package frc.robot.subsystems.transfer;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units;

public class TransferIOReal implements TransferIO{
    private final TalonFX transferMotor=new TalonFX(TransferConstants.TRANSFER_MOTOR_ID);

    private final DutyCycleOut percentRequest = new DutyCycleOut(0.0);
    
    private final StatusSignal<?> position;
    private final StatusSignal<?> velocity;
    private final StatusSignal<?> volts;
    private final StatusSignal<?> current;

    private final NeutralOut neutralRequest = new NeutralOut();//makes a neutral request
    private final DutyCycleOut percentRequest = new DutyCycleOut(0.5);

    public TransferIOReal(){
        configureMotor();
        position=transferMotor.getPosition();
        velocity=transferMotor.getVelocity();
        volts=transferMotor.getMotorVoltage();
        current=transferMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(TransferConstants.STATUS_UPDATE_HZ,position,velocity,volts,current);
    }
    @Override
    public void updateInputs(TransferIOInputs inputs){
        BaseStatusSignal.refreshAll(position,velocity,volts,current);
        inputs.positionRad=Units.rotationsToRadians(position.getValueAsDouble());
        inputs.velocityRpm=velocity.getValueAsDouble()*60.0;
        inputs.appliedVolts=volts.getValueAsDouble();
        inputs.supplyCurrentAmps=current.getValueAsDouble();

    }
    @Override
    public void setPercent(double percent){//for percent->use duty cycle for phoenix6 talonfx, use setControl
        transferMotor.setControl(percentRequest);
    }
    @Override
    public void stop(){
        transferMotor.setControl(neutralRequest);//tells motor to go neutral
    }
    private void configureMotor(){
        TalonFXConfiguration config=new TalonFXConfiguration(); //object containing all desired motor settings
        config.MotorOutput.NeutralMode=NeutralModeValue.Brake;//what it actually does at neutral
        //Other things that may be set here: Current limits, inverted vs not inverted, etc.
        transferMotor.getConfigurator().apply(config);//would use tryuntilok or timeout seconds but lazy
    }

}
