package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.configs.TalonFXConfiguration

public class TransferIOReal implements TransferIO{
private final TalonFX transferMotor = new TalonFX(TransferConstants.TRANSFER_MOTOR_ID);


    public TransferIOReal(){
    }
    @Override
    public void updateInputs(TransferIOInputs inputs){
    }
    @Override
    public void setPercent(double percent){//for percent->use duty cycle for phoenix6 talonfx, use setControl
    }
    @Override
    public void stop(){
    }
    private void configureMotor(){
        TalonFxConfiguration config = new TalonFxConfiguration();

    }

}
