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

public class Arm extends SubsystemBase {

  private TalonFX arm = new TalonFX(Constants.kArmMotor);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private PIDController armController = new PIDController(Constants.kArmKP, 0, Constants.kArmKD);

  private double armSetpoint = GetAngle(); // Set to current encoder value so elevetor doesnt "snap" when first enabled

  /** Creates a new Arm. */
  public Arm() {
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    arm.clearStickyFaults();
    arm.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate pid
    double pid = armController.calculate(GetAngle(), armSetpoint);
    pid = MathUtil.clamp(pid, -1 * Constants.kArmSpeedMax, Constants.kArmSpeedMax);
    arm.set(pid);

    SmartDashboard.putNumber("Arm PID Input", pid);
    SmartDashboard.putNumber("Arm Setpoint", armSetpoint);
    SmartDashboard.putNumber("Arm Encoder", GetAngle());
  }

  public void Set(double setpoint) {
    armSetpoint = setpoint;
  }

  public void ManualMovement(double input, double sensitivity) {
    armSetpoint = armSetpoint + input * sensitivity;
  }

  public void Stop() {
    armSetpoint = GetAngle();
  }

  public double GetAngle() {
    return arm.getPosition().getValueAsDouble();
  }
}
