// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import java.util.function.Supplier;

// import com.ctre.phoenix.led.*;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.CANdle.VBatOutputMode;
// import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
// import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
// import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
// import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;


// public class LightSubsystem extends SubsystemBase
// {

//     private final CANdle m_candle = new CANdle(3, "rio");
//     private final int LedCount = 31;

//     private Supplier<String> colorNameSupplier;

//     private enum Color {
//         Yellow,
//         Purple,
//         Rainbow
//     }

//     private Animation rainbow = new RainbowAnimation(1,0.5,LedCount);// {{
//     //    setNumLed(LedCount);
//     //}};

//     private Animation purple = new StrobeAnimation(255, 0, 255,0,0.01,LedCount);// {{
//         //setNumLed(LedCount);
//     //}};

//     private Animation yellow = new StrobeAnimation(255, 255, 0,0,0.01,LedCount);// {{
//         //setNumLed(LedCount);
//     //}};

//     private Color color = Color.Yellow;

//     public LightSubsystem() {
//         purple.setNumLed(LedCount);
//         yellow.setNumLed(LedCount);
//         rainbow.setNumLed(LedCount);
//         CANdleConfiguration config = new CANdleConfiguration();
//         config.statusLedOffWhenActive=true;
//         config.disableWhenLOS = false;
//         config.stripType=LEDStripType.GRB;
//         config.brightnessScalar = 0.1;

//         config.vBatOutputMode = VBatOutputMode.Modulated;
//         m_candle.configAllSettings(config,100);
//         //config.;
//         colorNameSupplier = new Supplier<String>() {
//             public String get() {
//                 switch (color) {
//                     case Yellow:
//                         return "Yellow Cone";
                        
                
//                     case Purple:
//                         return "Purple Cube";
//                     default:
//                         return "RGB GAMING ROBOT!!";
                        
//                 }
//             }
            
//         };
//         Shuffleboard.getTab("Game Tab").addString("color", colorNameSupplier);
//     }

//     public void playAuto() {
//         m_candle.animate(rainbow);
//     }
    
//     public void cycleColor() {
//         switch (color) {
//             case Yellow:
//                 color = Color.Purple;
//                 DriverStation.reportError("Error: " + m_candle.animate(purple), false);
                
//                 //m_candle.setLEDs(255,0, 255);
//                 break;
//             case Purple:
//                 color = Color.Yellow;
//                 m_candle.animate(yellow);
//                 //m_candle.setLEDs(255,255, 0);
//                 break;
//                     }
//     }


//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
        
//     }

//     @Override
//     public void simulationPeriodic() {
//         // This method will be called once per scheduler run during simulation
//     }
// }

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class LightSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(3, "rio");
    private final int LedCount = 40;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }
    Animation a;
    private AnimationTypes m_currentAnimation = AnimationTypes.Fire;

    public LightSubsystem() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Off;
        m_candle.configAllSettings(configAll, 100);
        changeAnimation(AnimationTypes.SetAll);
        CANdleFaults faults = new CANdleFaults();
        ErrorCode faultsError = m_candle.getFaults(faults);
        DriverStation.reportError("Error code: " + faultsError, false);
        a = new StrobeAnimation(240, 10, 180, 0, 0, LedCount);
    }

    public void incrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Fire); break;
            case Fire: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Strobe); break;
            case Strobe: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
            case Fire: changeAnimation(AnimationTypes.ColorFlow); break;
            case Larson: changeAnimation(AnimationTypes.Fire); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Strobe: changeAnimation(AnimationTypes.SingleFade); break;
            case Twinkle: changeAnimation(AnimationTypes.Strobe); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // if(m_toAnimate == null) {
        //     m_candle.setLEDs((int)(1 * 255), 
        //                       (int)(1 * 255), 
        //                       (int)(1 * 255));
        // } else {
            // m_candle.animate(m_toAnimate);
        // }
        // m_candle.modulateVBatOutput(1);
        // m_candle.setLEDs(0, 0, 255, 0, 0, 20);
        m_candle.animate(a);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}