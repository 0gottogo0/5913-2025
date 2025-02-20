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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private SparkMax intake = new SparkMax(kIntakeMotor, MotorType.kBrushless);
  private SparkMaxConfig cfg = new SparkMaxConfig();
  
  private PneumaticHub PH = new PneumaticHub(kPHID);
  
  private DoubleSolenoid clawSolenoid = PH.makeDoubleSolenoid(kClawClose, kClawOpen);
  
  private DigitalInput beamBreak = new DigitalInput(kBeamBreak);

  private Timer intakeTimer = new Timer();

  /** Creates a new Claw. */
  public Intake() {
    cfg
      .inverted(false)
      .idleMode(IdleMode.kCoast);

    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    
    intakeTimer.stop();
    intakeTimer.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Claw Is Loaded", GetBeamBreak());
  }

  public void RunIntake() {
    intake.set(kIntakeSpeed);
  }

  public  void RunIntakeReverse() {
    intake.set(kIntakeSpeedMax);
  }

  public void RunIntakeWithBeam() {
    if (GetBeamBreak()) {
      intakeTimer.start();
      Stop();
    } else if (intakeTimer.get() < 0.8 && intakeTimer.get() > 0.0) {
      Stop();
    } else {
      intake.set(kIntakeSpeed);
    }
  }

  public void EjectAlgae() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    intake.set(kIntakeSpeedMax);
  }

  public void Open(boolean algae) {
    if (!algae) {
      clawSolenoid.set(DoubleSolenoid.Value.kForward);
    } else {
      clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  // Whenever GetBeamBreak is true, there is a coral (or algae) in the claw
  public boolean GetBeamBreak() {
    return !beamBreak.get();
  }

  public void Stop() {
    intake.set(0);
  }
}
