// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

  private TalonFX arm = new TalonFX(kArmMotor);

  private TalonFXConfiguration cfg = new TalonFXConfiguration();

  private PIDController armController = new PIDController(kArmKP, 0, kArmKD);

  private DutyCycleEncoder armEncoder = new DutyCycleEncoder(kArmEncoderID);

  private double armSetpoint;
  private double armSetpointFinal;

  Elevator elevator = new Elevator();

  /** Creates a new Arm. */
  public Arm() {
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    arm.clearStickyFaults();
    arm.getConfigurator().apply(cfg);

    armSetpoint = GetAngle().in(Degrees); // Set to current encoder value so elevetor doesnt "snap" when first enabled

    armController.setTolerance(kArmTolerance);

    new Thread(() -> {
      try {
          Thread.sleep(3000);
          armSetpoint = GetAngle().in(Degree);
      } catch (Exception e) {
      }
    }).start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Add no-go zones
    if(elevator.GetPosition() > kArmNoGoZone) {
      armSetpointFinal = MathUtil.clamp(armSetpoint, kArmNoGoZoneLow, kArmNoGoZoneHigh);
    } else {
      armSetpointFinal = armSetpoint;
    }

    // Calculate pid
    double pid = armController.calculate(GetAngle().in(Degrees), armSetpointFinal);

    pid = MathUtil.clamp(pid, -1 * kArmSpeedMax, kArmSpeedMax);
    arm.set(-1 * pid);

    SmartDashboard.putNumber("Arm PID Input", pid);
    SmartDashboard.putNumber("Arm Setpoint", armSetpoint);
    SmartDashboard.putNumber("Arm Setpoint Final", armSetpointFinal);
    SmartDashboard.putNumber("Arm Encoder", GetAngle().in(Degrees));
  }

  public void Set(double setpoint) {
    armSetpoint = setpoint;
  }

  public void ManualMovement(double input, double sensitivity, boolean rawMode) {
    if (rawMode) {
      arm.set(input);
    } else {
      armSetpoint = armSetpoint + input * sensitivity;
    }
  }

  public void Stop() {
    armSetpoint = GetAngle().in(Degrees);
  }

  public Angle GetAngle() {
    return Rotations.of(armEncoder.get()).minus(Degrees.of(kArmEncoderOffset));
  }
}
