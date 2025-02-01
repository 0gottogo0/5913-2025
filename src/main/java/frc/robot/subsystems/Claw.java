// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  private SparkMax claw = new SparkMax(Constants.kClawMotor, MotorType.kBrushless);
  private DigitalInput beamBreak = new DigitalInput(Constants.kBeamBreak);
  private DoubleSolenoid clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kClawClose, Constants.kClawOpen);

  private SparkMaxConfig cfg = new SparkMaxConfig();

  /** Creates a new Claw. */
  public Claw() {
    cfg
      .inverted(false)
      .idleMode(IdleMode.kCoast);

    clawSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Claw Is Loaded", beamBreak.get());
  }

  public void Intake(boolean outake) {
    if (outake) {
      claw.set(-1 * Constants.kClawSpeedMax);
    } else if (!beamBreak.get()) {
      claw.set(Constants.kClawSpeedMax);
    } else {
      claw.set(Constants.kClawSpeedLow);
    }
  }

  public void Open(boolean close) {
    if (!close) {
      clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    } else {
      clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void Stop() {
    claw.set(0);
  }
}
