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

public class Wrist extends SubsystemBase {

private TalonFX wrist = new TalonFX(Constants.kWristMotor);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private PIDController wristController = new PIDController(Constants.kWristKP, 0, Constants.kWristKD);

  private double wristSetpoint = GetAngle(); // Set to current encoder value so elevetor doesnt "snap" when first enabled

  /** Creates a new Wrist. */
  public Wrist() {
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    wrist.clearStickyFaults();
    wrist.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate pid
    double pid = wristController.calculate(GetAngle(), wristSetpoint);
    pid = MathUtil.clamp(pid, -1 * Constants.kWristSpeedMax, Constants.kWristSpeedMax);
    wrist.set(pid);

    SmartDashboard.putNumber("Wrist PID Input", pid);
    SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint);
    SmartDashboard.putNumber("Wrist Encoder", GetAngle());
  }

  public void Set(double setpoint) {
    wristSetpoint = setpoint;
  }

  public void ManualMovement(double input, double sensitivity) {
    wristSetpoint = wristSetpoint + input * sensitivity;
  }

  public void Stop() {
    wristSetpoint = GetAngle();
  }

  public double GetAngle() {
    return wrist.getPosition().getValueAsDouble();
  }
}
