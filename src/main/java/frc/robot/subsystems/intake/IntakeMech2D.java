// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class creates a 2d mechanism that can be see on the robot in advantage scope.
 */
public class IntakeMech2D extends SubsystemBase {
  @AutoLogOutput(key = "Intake/IntakeMech2D")
  LoggedMechanism2d intakeBase;
  LoggedMechanismRoot2d intakeRoot;
  LoggedMechanismLigament2d intakeExtension;
  LoggedMechanismLigament2d intakeEnd;
  Intake intakeSubsystem;

  public IntakeMech2D(Intake intake) {
    intakeBase = new LoggedMechanism2d(Inches.of(-26), Inches.of(28.5));
    intakeRoot = intakeBase.getRoot("Intake", 0, 0.2);
    intakeExtension = intakeRoot.append(new LoggedMechanismLigament2d("IntakeLigament", Inches.of(10), Degrees.of(0), 5, new Color8Bit(Color.kDarkGreen)));
    intakeEnd = intakeExtension.append(new LoggedMechanismLigament2d("IntakeEnd", Inches.of(0.1), Degrees.of(0), 40, new Color8Bit(Color.kDimGray)));

    intakeSubsystem = intake;
  }

  @Override
  public void periodic() {
    intakeExtension.setLength(intakeSubsystem.getExtensionLength());
    intakeEnd.setColor(fromDutyCycle(intakeSubsystem.getRollerSpeed()));
  }

  /**
   * Created a scaling red or green value based on the duty cycle value. Gray is returned if the value is 0.
   * @param dutyCycle the [-1, 1] value you want to represent as a color
   */
  public static Color8Bit fromDutyCycle(double dutyCycle) {
    // If duty cycle is zero, show gray
    if (dutyCycle == 0) {
      return new Color8Bit(Color.kDimGray);
    }

    var h = dutyCycle < 0 ? 0 : 65; // red or green if negative or positive respectively
    var s = (int) Math.floor(Math.max(100.0, Math.abs(dutyCycle) * 255.0));

    return new Color8Bit(Color.fromHSV(h, s, 255));
  }
}
