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

  private SparkMax intake = new SparkMax(MotorIDs.Misc.INTAKE_MOTOR, MotorType.kBrushless);
  private SparkMaxConfig cfg = new SparkMaxConfig();
  
  private PneumaticHub PH = new PneumaticHub(PneumaticsIDs.PNEUMATICS_HUB);
  
  private DoubleSolenoid clawSolenoid = PH.makeDoubleSolenoid(PneumaticsIDs.CLAW_CLOSE, PneumaticsIDs.CLAW_OPEN);
  
  private DigitalInput beamBreak = new DigitalInput(IO.Misc.BEAM_BREAK);

  private Timer ejectTimer = new Timer();

  public boolean ignoreBeamBreak = false;
  public boolean holdAlgae = false; // Turn this varable true if we are in an algae spot
  public boolean groundCoral = false;
  public boolean ejectCoral = false;
  public boolean levelOne = false;

  /** Creates a new Claw. */
  public Intake() {
    cfg
      .inverted(false)
      .idleMode(IdleMode.kCoast);

    clawSolenoid.set(DoubleSolenoid.Value.kForward);

    ejectTimer.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (ejectCoral) {
      Stop();
    }

    // Debug
    SmartDashboard.putBoolean("Claw Is Loaded", GetBeamBreak());
  }

  // Run the intake without the beambreak
  public void RunIntake() {
    intake.set(-Speeds.INTAKE_SPEED_MAX);
  }

  public void RunIntakeSlow() {
    intake.set(-Speeds.INTAKE_SPEED_L1);
  }

  // Run the intake backwards
  public void RunIntakeReverse() {
    intake.set(Speeds.INTAKE_SPEED_MAX);
  }

  // Run the intake with the beambreak
  public void RunIntakeWithBeam() {
    if (groundCoral) {
      RunIntakeReverse();
      return;
    }

    if (levelOne) {
      RunIntakeSlow();
      return;
    }

    if (!ignoreBeamBreak) {
      if (!GetBeamBreak()) {
        intake.set(-Speeds.INTAKE_SPEED);
      } else {
        Stop();
      }
    } else {
      RunIntake();
    }
  }

  // Eject the Algae by closing the claw and outtake
  public void EjectAlgae() {
    ejectCoral = true;
    holdAlgae = false;
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    intake.set(Speeds.INTAKE_SPEED_MAX);
  }

  /**
   * Open or close claw
   * @param algae false = close claw
   */
  public void Open(boolean algae) {
    groundCoral = false;
    levelOne = false;
    if (!algae) {
      clawSolenoid.set(DoubleSolenoid.Value.kForward);
      holdAlgae = false;
      Stop();
    } else {
      clawSolenoid.set(DoubleSolenoid.Value.kReverse);
      holdAlgae = true;
      Stop();
    }
  }

  public void OpenNoAlgae(boolean ground) {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
      holdAlgae = false;
      levelOne = false;
      if(ground) {
        groundCoral = true;
      } else {
        groundCoral = false;
      }
      Stop();
  }

  public void CloseNoAlgae() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
    holdAlgae = false;
    groundCoral = false;
    levelOne = true;
    Stop();
  }

  /**
   * Get the status of the beambreak
   * @return beambreak status, true = coral in claw
   */
  public boolean GetBeamBreak() {
    return !beamBreak.get();
  }

  // Stop the intake
  public void Stop() {
    if (ejectCoral) {
      ejectTimer.start();
      if (ejectTimer.get() > 0.75) {
        ejectTimer.stop();
        ejectTimer.reset();
        ejectCoral = false;
        intake.set(0);
      }

      return;
    }

    if (groundCoral) {
      intake.set(-Speeds.INTAKE_SPEED_HOLD_ALGAE);
      return;
    }

    if (levelOne) {
      intake.set(-Speeds.INTAKE_SPEED_HOLD_ALGAE);
      return;
    }

    if (holdAlgae) {
      intake.set(Speeds.INTAKE_SPEED_HOLD_ALGAE);
      return;
    }
    
    intake.set(0);
  }
}
