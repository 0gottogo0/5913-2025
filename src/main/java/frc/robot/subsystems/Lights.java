// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

  private CANdle candle = new CANdle(IO.CANdle.kCANdleID);

  private CANdleConfiguration cfg = new CANdleConfiguration();

  private Animation m_toAnimate = null;

  // Create our states
  public enum AnimationTypes {
    IDLE,
    DISABLED,
    AUTONOMOUS,
    ENABLED,
    BEAMBREAK,
    TRACK,
    ALGAE,
    ESTOP
  }

  private AnimationTypes m_currentAnimation;

  private Camera camera;
    
  private Intake intake;

  /** Creates a new Lights. */
  public Lights(Camera camera, Intake intake) {
    this.camera = camera;
    this.intake = intake;
    cfg.statusLedOffWhenActive = true;
    cfg.stripType = LEDStripType.GRB;
    cfg.disableWhenLOS = false;
    
    candle.configFactoryDefault();
    candle.clearStickyFaults();
    candle.configAllSettings(cfg);
    changeAnimation(AnimationTypes.IDLE);
  }

  // Needed for CANdle
  public void incrementAnimation() {
    switch(m_currentAnimation) {
      case IDLE: changeAnimation(AnimationTypes.DISABLED); break;
      case DISABLED: changeAnimation(AnimationTypes.ENABLED); break;
      case AUTONOMOUS: changeAnimation(AnimationTypes.BEAMBREAK); break;
      case ENABLED: changeAnimation(AnimationTypes.AUTONOMOUS); break;
      case BEAMBREAK: changeAnimation(AnimationTypes.TRACK); break;
      case TRACK: changeAnimation(AnimationTypes.ALGAE); break;
      case ALGAE: changeAnimation(AnimationTypes.ESTOP); break;
      case ESTOP: changeAnimation(AnimationTypes.IDLE); break;
    }
  }

  public void decrementAnimation() {
    switch(m_currentAnimation) {
      case IDLE: changeAnimation(AnimationTypes.ESTOP); break;
      case DISABLED: changeAnimation(AnimationTypes.IDLE); break;
      case AUTONOMOUS: changeAnimation(AnimationTypes.DISABLED); break;
      case ENABLED: changeAnimation(AnimationTypes.AUTONOMOUS); break;
      case BEAMBREAK: changeAnimation(AnimationTypes.ENABLED); break;
      case TRACK: changeAnimation(AnimationTypes.BEAMBREAK); break;
      case ALGAE: changeAnimation(AnimationTypes.TRACK); break;
      case ESTOP: changeAnimation(AnimationTypes.ALGAE); break;
    }
  }

  // Coralate states to animations
  public void changeAnimation(AnimationTypes toChange) {
    m_currentAnimation = toChange;
      switch(toChange) {
        case IDLE:
          m_toAnimate = new ColorFlowAnimation(255, 255, 255);
          break;
        case DISABLED:
          m_toAnimate = new RainbowAnimation(1, 0.4, IO.CANdle.kTotalLightAmount);
          break;
        case AUTONOMOUS:
          m_toAnimate = new FireAnimation();
          break;
        case ENABLED:
          m_toAnimate = null;
          if (isBlue()) {
            candle.setLEDs(0, 0, 255);
          } else {
            candle.setLEDs(255, 0, 0);
          }
          break;
        case BEAMBREAK:
          m_toAnimate = null;
          candle.setLEDs(255, 255, 0);
          break;
        case TRACK:
          m_toAnimate = new ColorFlowAnimation(0, 255, 0);
          break;
        case ALGAE:
          m_toAnimate = null;
          candle.setLEDs(0, 255, 255, 0, 0, IO.CANdle.kTotalLightAmount);
          break;
        case ESTOP:
          m_toAnimate = new StrobeAnimation(255, 0, 0);
          break;
      }
    }

  // Get if we are on the blue side or red
  public boolean isBlue() {
    try {
      if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
        return true;
      } else {
        return false;
      }
    } catch (Exception e) {
      return false;      
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Set the lights to our predefinded states
    if (!DriverStation.isDSAttached()) {
      changeAnimation(AnimationTypes.IDLE);
    } else if (DriverStation.isEStopped()) {
      changeAnimation(AnimationTypes.ESTOP);
    } else if (DriverStation.isDSAttached() && DriverStation.isDisabled()) {
      changeAnimation(AnimationTypes.DISABLED);
    } else if (DriverStation.isDSAttached() && DriverStation.isAutonomousEnabled()) {
      changeAnimation(AnimationTypes.AUTONOMOUS);
    } else if (intake.holdAlgae) {
      changeAnimation(AnimationTypes.ALGAE);
    } else if (camera.IsTracking()) {
      changeAnimation(AnimationTypes.TRACK);
    } else if (intake.GetBeamBreak()) {
      changeAnimation(AnimationTypes.BEAMBREAK);
    } else if (DriverStation.isEnabled()) {
      changeAnimation(AnimationTypes.ENABLED);
    } else {
      changeAnimation(AnimationTypes.IDLE);
    }
    
    // Debug
    SmartDashboard.putString("animation", m_currentAnimation.toString());

    // Needed for CANdle
    if(m_toAnimate == null) {
      candle.animate(null);
    } else {
      candle.animate(m_toAnimate);
    }
  }
}
