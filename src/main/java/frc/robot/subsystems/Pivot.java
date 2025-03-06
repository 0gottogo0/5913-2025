// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Speeds;

public class Pivot extends SubsystemBase {

  private TalonFX pivotLeftMaster = new TalonFX(MotorIDs.Misc.kPivotLeftMotor);
  private TalonFXConfiguration cfgLeft = new TalonFXConfiguration();

  private TalonFX pivotRightFollower = new TalonFX(MotorIDs.Misc.kPivotRightMotor);
  private TalonFXConfiguration cfgRight = new TalonFXConfiguration();

  private PIDController pivotController = new PIDController(PID.Pivot.kPivotKP, 0, 0);

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IO.Misc.kPivotEncoderID);
  
  private double pivotSetpoint;
  private boolean pidToggle;

  /** Creates a new Pivot. */
  public Pivot() {
    cfgLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfgLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    cfgRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotLeftMaster.clearStickyFaults();
    pivotLeftMaster.getConfigurator().apply(cfgLeft);

    pivotRightFollower.clearStickyFaults();
    pivotRightFollower.getConfigurator().apply(cfgRight);

    // Set the right motor to follow the left one
    pivotRightFollower.setControl(new Follower(pivotLeftMaster.getDeviceID(), false));

    pivotSetpoint = GetAngle().in(Degrees); // Set to current encoder value so elevetor doesnt "snap" when first enabled

    pidToggle = true;

    // Wait to set current encoder value, sometimes it takes a tinsie bit
    new Thread(() -> {
      try {
          Thread.sleep(3000);
          pivotSetpoint = GetAngle().in(Degree);
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
      pid = pivotController.calculate(GetAngle().in(Degrees), pivotSetpoint);
    }
    
    pid = MathUtil.applyDeadband(MathUtil.clamp(pid, -1 * Speeds.kPivotSpeedMax, Speeds.kPivotSpeedMax), PID.Pivot.kPivotDeadzone);
    pivotLeftMaster.set(-1 * pid);

    // Debug
    SmartDashboard.putNumber("Pivot PID Input", pid);
    SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
    SmartDashboard.putNumber("Pivot Encoder", GetAngle().in(Degrees));
    SmartDashboard.putBoolean("Pivot Encoder Status", GetAngle().in(Degree) != 360); // Returns false if roborio gets shorted or encoder gets unplugged
  }

  // Set the setpoint
  public void Set(double setpoint) {
    pivotSetpoint = setpoint;
  }

  // Move the pivot manually with the pid
  // Move the pivot manually without the pid if rawMode is true
  public void ManualMovement(double input, double sensitivity, boolean rawMode) {
    if (rawMode) {
      pivotLeftMaster.set(input);
      pidToggle = false;
    } else {
      pivotSetpoint = pivotSetpoint + input * sensitivity;
    }
  }

  // Stop pivot
  public void Stop() {
    pivotSetpoint = GetAngle().in(Degrees);
    pidToggle = true;
  }

  // Get external encoder position
  public Angle GetAngle() {
    return Rotations.of(pivotEncoder.get()).minus(Degrees.of(IO.Misc.kPivotEncoderOffset));
  }

  // Get the current setpoint for the pid controller
  // This helps with the beambreak in Robot.java
  public double GetSetpoint() {
    return pivotController.getSetpoint();
  }
}