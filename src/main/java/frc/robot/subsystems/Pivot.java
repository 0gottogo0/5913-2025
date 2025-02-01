// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

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
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

  private TalonFX pivotLeftMaster = new TalonFX(Constants.kPivotLeftMotor);
  private TalonFX pivotRightFollower = new TalonFX(Constants.kPivotRightMotor);

  private TalonFXConfiguration cfgLeft = new TalonFXConfiguration();
  private TalonFXConfiguration cfgRight = new TalonFXConfiguration();

  private PIDController pivotController = new PIDController(Constants.kPivotKP, 0, Constants.kPivotKD);

  private double pivotSetpoint;

  private DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(Constants.pivotEncoderID);

  /** Creates a new Pivot. */
  public Pivot() {
    cfgLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfgLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    cfgRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    

    pivotLeftMaster.clearStickyFaults();
    pivotLeftMaster.getConfigurator().apply(cfgLeft);

    pivotRightFollower.clearStickyFaults();
    pivotRightFollower.getConfigurator().apply(cfgRight);

    pivotRightFollower.setControl(new Follower(pivotLeftMaster.getDeviceID(), false));

    pivotSetpoint = GetAngle().in(Degrees); // Set to current encoder value so elevetor doesnt "snap" when first enabled

    pivotController.setTolerance(Constants.kPivotTolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pid = 0;
    // Calculate pid
    if(!pivotController.atSetpoint()) {
      pid = pivotController.calculate(GetAngle().in(Degrees), pivotSetpoint);
    }
    
    pid = MathUtil.clamp(pid, -1 * Constants.kPivotSpeedMax, Constants.kPivotSpeedMax);
    pivotLeftMaster.set(pid);

    SmartDashboard.putNumber("Pivot PID Input", pid);
    SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
    SmartDashboard.putNumber("Pivot Encoder", GetAngle().in(Degrees));
  }

  public void Set(double setpoint) {
    pivotSetpoint = setpoint;
  }

  public void ManualMovement(double input, double sensitivity) {
    pivotSetpoint = pivotSetpoint + input * sensitivity;
  }

  public void Stop() {
    pivotSetpoint = GetAngle().in(Degrees);
  }

  // Average out both encoders
  public Angle GetAngle() {
    return Rotations.of(pivotEncoder.get()).minus(Degrees.of(0)); // TODO: config offset
    }
}