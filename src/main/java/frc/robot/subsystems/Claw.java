// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {

  private SparkMax claw = new SparkMax(kClawMotor, MotorType.kBrushless);
  private DigitalInput beamBreak = new DigitalInput(kBeamBreak);
  private PneumaticHub PH = new PneumaticHub(kPHID);
  private DoubleSolenoid clawSolenoid = PH.makeDoubleSolenoid(kClawClose, kClawOpen);

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

  public void Intake(boolean reverse) {
    if (reverse) {
      claw.set(-1 * kClawSpeedMax);
    } else {
      claw.set(kClawSpeedMax);
    }
  }

  public void Open(boolean alge) {
    if (!alge) {
      clawSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  public void Stop() {
    claw.set(0);
  }
}
