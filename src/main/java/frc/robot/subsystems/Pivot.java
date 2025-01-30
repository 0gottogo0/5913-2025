// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

  private TalonFX pivotLeft = new TalonFX(Constants.kPivotLeftMotor);
  private TalonFX pivotRight = new TalonFX(Constants.kPivotRightMotor);

  private TalonFXConfiguration cfgLeft = new TalonFXConfiguration();
  private TalonFXConfiguration cfgRight = new TalonFXConfiguration();

  private PIDController pivotController = new PIDController(Constants.kPivotKP, 0, Constants.kPivotKD);

  private double pivotSetpoint = GetAngle(); // Set to current encoder value so elevetor doesnt "snap" when first enabled

  /** Creates a new Pivot. */
  public Pivot() {
    cfgLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfgLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    cfgRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfgRight.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotLeft.clearStickyFaults();
    pivotLeft.getConfigurator().apply(cfgLeft);

    pivotRight.clearStickyFaults();
    pivotRight.getConfigurator().apply(cfgRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate pid
    double pid = pivotController.calculate(GetAngle(), pivotSetpoint);
    pid = MathUtil.clamp(pid, -1 * Constants.kPivotSpeedMax, Constants.kPivotSpeedMax);
    pivotLeft.set(pid);
    pivotRight.set(pid);

    SmartDashboard.putNumber("Pivot PID Input", pid);
    SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
    SmartDashboard.putNumber("Pivot Encoder Left", pivotLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Encoder Right", pivotRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Encoder Average", GetAngle());
  }

  public void Set(double setpoint) {
    pivotSetpoint = setpoint;
  }

  public void ManualMovement(double input, double sensitivity) {
    pivotSetpoint = pivotSetpoint + input * sensitivity;
  }

  public void Stop() {
    pivotSetpoint = GetAngle();
  }

  // Average out both encoders
  public double GetAngle() {
    return pivotLeft.getPosition().getValueAsDouble() + pivotRight.getPosition().getValueAsDouble() /2;
  }
}