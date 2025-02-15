// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

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

public class Wrist extends SubsystemBase {

  private SparkMax wrist = new SparkMax(kWristMotor, MotorType.kBrushless);

  private SparkMaxConfig cfg = new SparkMaxConfig();

  private PIDController wristController = new PIDController(kWristKP, 0, kWristKD);

  private double wristSetpoint;

  private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(kWristEncoderID);

  /** Creates a new Wrist. */
  public Wrist() {
    cfg
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    wristSetpoint = GetAngle().in(Degree);

    wristController.setTolerance(kWristTolerance);

    new Thread(() -> {
      try {
          Thread.sleep(3000);
          wristSetpoint = GetAngle().in(Degree);
      } catch (Exception e) {
      }
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double pid = 0;

    // Calculate pid
    if(!wristController.atSetpoint()) {
      pid = wristController.calculate(GetAngle().in(Degree), wristSetpoint);
    }

    pid = MathUtil.clamp(pid, -1 * kWristSpeedMax, kWristSpeedMax);
    wrist.set(-1 * pid);
    
    SmartDashboard.putNumber("Wrist PID Input", pid);
    SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint);
    SmartDashboard.putNumber("Wrist Encoder", GetAngle().in(Degree));
  }

  public void Set(double setpoint) {
    wristSetpoint = setpoint;
  }

  public void ManualMovement(double input, double sensitivity, boolean rawMode) {
    wristSetpoint = wristSetpoint + input * sensitivity;
  }

  public void Stop() {
    wristSetpoint = GetAngle().in(Degrees);
  }

  public Angle GetAngle() {
    return Rotations.of(wristEncoder.get()).minus(Degrees.of(kWristEncoderOffset));
  }
}
