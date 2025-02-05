// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {

  private SparkMax wrist = new SparkMax(Constants.kWristMotor, MotorType.kBrushless);

  private SparkMaxConfig cfg = new SparkMaxConfig();

  private PIDController wristController = new PIDController(Constants.kWristKP, 0, Constants.kWristKD);

  private double wristSetpoint;

  private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(Constants.kWristEncoderID);

  /** Creates a new Wrist. */
  public Wrist() {
    cfg
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    wristSetpoint = GetAngle().in(Degree);

    wristController.setTolerance(Constants.kWristTolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double pid = 0;

    // Calculate pid
    if(!wristController.atSetpoint()) {
      pid = wristController.calculate(GetAngle().in(Degree), wristSetpoint);
    }
    pid = MathUtil.clamp(pid, -1 * Constants.kWristSpeedMax, Constants.kWristSpeedMax);
    wrist.set(pid);

    SmartDashboard.putNumber("Wrist PID Input", pid);
    SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint);
    SmartDashboard.putNumber("Wrist Encoder", GetAngle().in(Degree));
  }

  public void Set(double setpoint) {
    wristSetpoint = setpoint;
  }

  public void ManualMovement(double input, double sensitivity) {
    wristSetpoint = wristSetpoint + input * sensitivity;
  }

  public void Stop() {
    wristSetpoint = GetAngle().in(Degrees);
  }

  public Angle GetAngle() {
    return Rotations.of(wristEncoder.get()).minus(Degrees.of(Constants.kWristEncoderOffset));
  }
}
