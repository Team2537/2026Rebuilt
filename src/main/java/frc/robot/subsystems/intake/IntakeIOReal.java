package frc.robot.subsystems.intake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

import main.java.frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);
    private final TalonFX leftIntakeMotor = new TalonFX(IntakeConstants.LEFT_MOTOR_ID);
    private final TalonFX rightIntakeMotor = new TalonFX(IntakeConstants.RIGHT_MOTOR_ID);

    private final VelocityTorqueCurrentFOC rollerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

    private final NeutralOut neutralRequest = new NeutralOut();

    private final StatusSignal<?> leftPosition;
    private final StatusSignal<?> leftVelocity;
    private final StatusSignal<?> leftAppliedVolts;
    private final StatusSignal<?> leftSupplyCurrent;

    private final StatusSignal<?> rightPosition;
    private final StatusSignal<?> rightVelocity;
    private final StatusSignal<?> rightAppliedVolts;
    private final StatusSignal<?> rightSupplyCurrent;

    private final StatusSignal<?> rollerPosition;
    private final StatusSignal<?> rollerVelocity;
    private final StatusSignal<?> rollerAppliedVolts;
    private final StatusSignal<?> rollerSupplyCurrent;



    public IntakeIOReal() {
        
        
        leftPosition = leftIntakeMotor.getPosition();
        leftVelocity = leftIntakeMotor.getVelocity();
        leftAppliedVolts = leftIntakeMotor.getMotorVoltage();
        leftSupplyCurrent = leftIntakeMotor.getSupplyCurrent();

        rightPosition = rightIntakeMotor.getPosition();
        rightVelocity = rightIntakeMotor.getVelocity();
        rightAppliedVolts = rightIntakeMotor.getMotorVoltage();
        rightSupplyCurrent = rightIntakeMotor.getSupplyCurrent();

        rollerPosition = rollerMotor.getPosition();
        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVolts = rollerMotor.getMotorVoltage();
        rollerSupplyCurrent = rollerMotor.getSupplyCurrent();


        BaseStatusSignal.setUpdateFrequencyForAll(
            IntakeConstants.STATUS_UPDATE_HZ,
            leftPosition,
            leftVelocity,
            leftAppliedVolts,
            leftSupplyCurrent,
            rightPosition,
            rightVelocity,
            rightAppliedVolts,
            rightSupplyCurrent,
            rollerPosition,
            rollerVelocity,
            rollerAppliedVolts,
            rollerSupplyCurrent);
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftPosition,
            leftVelocity,
            leftAppliedVolts,
            leftSupplyCurrent,
            rightPosition,
            rightVelocity,
            rightAppliedVolts,
            rightSupplyCurrent,
            rollerPosition,
            rollerVelocity,
            rollerAppliedVolts,
            rollerSupplyCurrent);
        
        inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
        inputs.leftPositionRad = Units.rotationsToRadians(leftPosition.getValueAsDouble());
        inputs.leftCurrentAmps = leftSupplyCurrent.getValueAsDouble();
        inputs.leftVelocityRpm = leftVelocity.getValueAsDouble()*60.0;

        inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
        inputs.rightPositionRad = Units.rotationsToRadians(rightPosition.getValueAsDouble());
        inputs.rightCurrentAmps = rightSupplyCurrent.getValueAsDouble();
        inputs.rightVelocityRpm = rightVelocity.getValueAsDouble()*60.0;

        inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerPositionRad = Units.rotationsToRadians(rollerPosition.getValueAsDouble());
        inputs.rollerCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
        inputs.rollerVelocityRpm = rollerVelocity.getValueAsDouble()*60.0;
    }

    @Override
    public void setRollerRpm(double rpm) {
        rollerMotor.setControl(rollerVelocityRequest.withVelocity(rpm/60.0));
    }
    
    @Override
    public void retract(){
        
    }

    @Override
    public void extend(){
    }

    @Override
    public void stop(){
        rollerMotor.setControl(neutralRequest);
    }
}