// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private TalonFX elevator = new TalonFX(Constants.kElevatorMotor);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private ElevatorFeedforward elevatorController = new ElevatorFeedforward(Constants.kElevatorKS, Constants.kElevatorKG, Constants.kElevatorKV);

  private double elevatorSetpoint = GetAngle(); // Set to current encoder value so elevetor doesnt "snap" when first enabled

  /** Creates a new Elevator. */
  public Elevator() {
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevator.clearStickyFaults();
    elevator.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Calculate pid
    double pid = elevatorController.calculate(GetAngle(), elevatorSetpoint);
    pid = MathUtil.clamp(pid, -1 * Constants.kElevatorSpeedMax, Constants.kElevatorSpeedMax);
    elevator.set(pid);

    SmartDashboard.putNumber("Elevator PID Input", pid);
    SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpoint);
    SmartDashboard.putNumber("Elevator Encoder", GetAngle());
  }

  public void Set(double setpoint) {
    elevatorSetpoint = setpoint;
  }

  public void ManualMovement(double input, double sensitivity) {
    elevatorSetpoint = elevatorSetpoint + input * sensitivity;
  }

  public void Stop() {
    elevatorSetpoint = GetAngle();
  }

  public double GetAngle() {
    return elevator.getPosition().getValueAsDouble();
  }
}