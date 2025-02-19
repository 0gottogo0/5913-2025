// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private TalonFX elevator = new TalonFX(kElevatorMotor);
  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private PIDController elevatorController = new PIDController(kElevatorKP, 0, kElevatorKD);

  private double elevatorSetpoint; 
  private boolean pidToggle;

  /** Creates a new Elevator. */
  public Elevator() {
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevator.clearStickyFaults();
    elevator.getConfigurator().apply(cfg);

    elevatorSetpoint = GetPosition(); // Set to current encoder value so elevetor doesnt "snap" when first enabled

    pidToggle = true;

    elevatorController.setTolerance(kElevatorTolerance);

    new Thread(() -> {
      try {
          Thread.sleep(3000);
          elevatorSetpoint = GetPosition();
      } catch (Exception e) {
      }
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate pid
    double pid = elevatorController.calculate(GetPosition(), elevatorSetpoint);

    if (pidToggle) {
      pid = MathUtil.clamp(pid, -1 * kElevatorSpeedMax, kElevatorSpeedMax);
      elevator.set(pid);
    }

    SmartDashboard.putNumber("Elevator PID Input", pid);
    SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator Encoder", GetPosition());
  }

  public void Set(double setpoint) {
    elevatorSetpoint = setpoint;
  }

  public void ManualMovement(double input, double sensitivity, boolean rawMode) {
    if (rawMode) {
      elevator.set(input);
      pidToggle = false;
    } else {
      elevatorSetpoint = elevatorSetpoint + input * sensitivity;
    }
  }

  public void Stop() {
    elevatorSetpoint = GetPosition();
    pidToggle = true;
  }

  public double GetPosition() {
    return elevator.getPosition().getValueAsDouble();
  }
}