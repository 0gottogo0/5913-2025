// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

  private CANdle candle = new CANdle(kCANdleID);

  private CANdleConfiguration cfg = new CANdleConfiguration();

  private Animation m_toAnimate = null;

  public enum AnimationTypes {
      ColorFlow,
      Fire,
      Larson,
      Rainbow,
      RgbFade,
      SingleFade,
      Twinkle,
      TwinkleOff,
      SetAll
    }
    
    private Intake intake;
    private Camera camera;

    private AnimationTypes m_currentAnimation;

  /** Creates a new Lights. */
  public Lights(Intake intake, Camera camera) {
    this.intake = intake;
    cfg.statusLedOffWhenActive = true;
    cfg.stripType = LEDStripType.GRB;
    cfg.disableWhenLOS = false;
    
    candle.configFactoryDefault();
    candle.clearStickyFaults();
    candle.configAllSettings(cfg);
    setColors();
    changeAnimation(AnimationTypes.SetAll);
  }

  public void incrementAnimation() {
    switch(m_currentAnimation) {
        case ColorFlow: changeAnimation(AnimationTypes.Larson); break;
        case Larson: changeAnimation(AnimationTypes.Rainbow); break;
        case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
        case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
        case SingleFade: changeAnimation(AnimationTypes.Twinkle); break;
        case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
        case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
        case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    }
} 

public void decrementAnimation() {
    switch(m_currentAnimation) {
        case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
        case Larson: changeAnimation(AnimationTypes.ColorFlow); break;
        case Rainbow: changeAnimation(AnimationTypes.Larson); break;
        case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
        case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
        case Twinkle: changeAnimation(AnimationTypes.SingleFade); break;
        case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
        case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
    }
}

  public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

  public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.5, kTotalLightAmount);
                break;
            case SingleFade:
                if (camera.IsTracking() == true) {
                    m_toAnimate = new SingleFadeAnimation(0, 255, 0, 0, 0.5, kTotalLightAmount);
                } else {
                    if (isBlue()) {
                        m_toAnimate = new SingleFadeAnimation(0, 0, 200, 0, 0.5, kTotalLightAmount);
                    } else {
                        m_toAnimate = new SingleFadeAnimation(200, 0, 0, 0, 0.5, kTotalLightAmount);
                    }
                }
                break;
            case Fire:
                m_toAnimate = new FireAnimation();
                break;
            case SetAll:
                m_toAnimate = null;
                break;
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
            return false;      
    }
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!DriverStation.isDSAttached()) {
        changeAnimation(AnimationTypes.SingleFade);
    } else if (DriverStation.isDSAttached() && DriverStation.isDisabled()) {
        changeAnimation(AnimationTypes.Rainbow);
    } else if (DriverStation.isDSAttached() && DriverStation.isAutonomous()) {
        changeAnimation(AnimationTypes.Fire);
    } else if (camera.IsTracking() == true) {
        changeAnimation(AnimationTypes.SingleFade);
    } else if (intake.GetBeamBreak() == true) {
        candle.setLEDs(0, 255, 0, 0, 0, kTotalLightAmount);
    } else {
        setColors();
        if (isBlue()) { 
            candle.setLEDs(0, 0, 255, 0, 0, kTotalLightAmount);
        } else {
            candle.setLEDs(255, 0, 0, 0, 0, kTotalLightAmount);
        }
    }
    
    if(m_toAnimate == null) {
        candle.animate(null);
    } else {
        candle.animate(m_toAnimate);
    }

    SmartDashboard.putString("Current Animation", m_currentAnimation.toString());
}
}
