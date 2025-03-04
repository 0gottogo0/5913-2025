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

  private SparkMax wrist = new SparkMax(MotorIDs.Misc.kWristMotor, MotorType.kBrushless);

  private SparkMaxConfig cfg = new SparkMaxConfig();

  private PIDController wristController = new PIDController(PID.Wrist.kWristKP, 0, 0);
  
  private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(IO.Misc.kWristEncoderID);
  
  private double wristSetpoint;
  private boolean pidToggle;

  /** Creates a new Wrist. */
  public Wrist() {
    cfg
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    wristSetpoint = GetAngle().in(Degree); // Set to current encoder value so elevetor doesnt "snap" when first enabled

    pidToggle = true;

    // Wait to set current encoder value, sometimes it takes a tinsie bit
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

    if (pidToggle) {
      pid = wristController.calculate(GetAngle().in(Degree), wristSetpoint);
    }

    pid = MathUtil.clamp(pid, -1 * Speeds.kWristSpeedMax, Speeds.kWristSpeedMax);
    wrist.set(-1 * pid);
    
    // Debug
    SmartDashboard.putNumber("Wrist PID Input", pid);
    SmartDashboard.putNumber("Wrist Setpoint", wristSetpoint);
    SmartDashboard.putNumber("Wrist Encoder", GetAngle().in(Degree));
    SmartDashboard.putBoolean("Wrist Encoder Status", GetAngle().in(Degree) != 360); // Returns false if roborio gets shorted or encoder gets unplugged
  }

  // Set the setpoint
  public void Set(double setpoint) {
    wristSetpoint = setpoint;
  }

  // Move the wrist manually with the pid
  // Move the wrist manually without the pid if rawMode is true
  public void ManualMovement(double input, double sensitivity, boolean rawMode) {
    if (rawMode) {
      wrist.set(input);
      pidToggle = false;
    } else {
      wristSetpoint = wristSetpoint + input * sensitivity;
    }
  }

  // Stop pivot
  public void Stop() {
    wristSetpoint = GetAngle().in(Degrees);
    pidToggle = true;
  }

  // Get external encoder position
  public Angle GetAngle() {
    return Rotations.of(wristEncoder.get()).minus(Degrees.of(IO.Misc.kWristEncoderOffset));
  }

  // Get the current setpoint for the pid controller
  // This is currently not used to my knowlage but keep it here to be safe
  public double GetSetpoint() {
    return wristController.getSetpoint();
  }
}
