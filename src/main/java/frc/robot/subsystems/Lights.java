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
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IO;

public class Lights extends SubsystemBase {

  private CANdle candle = new CANdle(IO.CANdle.kCANdleID);

  private CANdleConfiguration cfg = new CANdleConfiguration();

  private Animation m_toAnimate = null;

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

  public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange) {
            case IDLE:
                m_toAnimate = new ColorFlowAnimation(255, 255, 255);
                break;
            case DISABLED:
                m_toAnimate = new RainbowAnimation();
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
        }
    }

  public boolean isBlue() {
    try {
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            return true;
        } else {
            return false;
        }
    } catch (Exception e) {
            //e.printStackTrace();     
            return false;      
    }
  }

  @Override
  public void periodic() {
    if (!DriverStation.isDSAttached())

    /*
    if (!DriverStation.isDSAttached()) {
        changeAnimation(AnimationTypes.ENABLED);
    } else if (DriverStation.isDSAttached() && DriverStation.isDisabled()) {
        changeAnimation(AnimationTypes.DISABLED);
    } else if (feeder.isfeedStopped() || shooter.isShooterUpToSpeed()) {
        setColors();
        candle.setLEDs(0, 255, 0);
    } else {
        setColors();
        if (isBlue()) { 
            candle.setLEDs(0, 0, 255, 0, 0, IO.CANdle.kTotalLightAmount);
            //setEveryOtherLED(255, 0, 0, underGlowLedCount);
        } else {
            candle.setLEDs(255, 0, 0, 0, 0, IO.CANdle.kTotalLightAmount);
            //setEveryOtherLED(0, 0, 255, underGlowLedCount);
        }
    }
    */
    
    // Debug
    SmartDashboard.putString("animation", m_currentAnimation.toString());

    // This method will be called once per scheduler run
    if(m_toAnimate == null) {
        candle.animate(null);
    } else {
        candle.animate(m_toAnimate);
    }
  }
}
