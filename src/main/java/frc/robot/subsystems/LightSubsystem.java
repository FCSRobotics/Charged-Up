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

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
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
    private final int LedCount = 32;

    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LedCount);

    private Animation m_toAnimate = null;

    public enum Color {
        Yellow,
        Purple,
        Off
    }

    private Color currentColor = Color.Yellow;
    Animation a;
    private AddressableLED ledStrip = new AddressableLED(0);

    private final int reciprocalOfBrightness = 2;
    private int rainbowFirstPixelHue;

    public LightSubsystem() {
        
        
        ledStrip.setLength(buffer.getLength());
        
        setAll(0, 0, 0);
        
        ledStrip.start();
    }

    public void rainbow() {
        for (int i = 0; i < buffer.getLength(); i ++) {
            final var hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
            buffer.setHSV(i, hue, 255 / reciprocalOfBrightness, 128);
            
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
        ledStrip.setData(buffer);
    }

    
    
    private final int dartSize = 7;
    private int dartTail = LedCount - dartSize - 1;
    private int dartHead = 0;
    private final int[] dartColorRGB = new int[] {255, 255, 255};

    public void dart() {
        
        

        dartTail++;
        dartHead++;
        
        dartTail %= LedCount - 1;
        dartHead %= LedCount - 1;
        
        buffer.setRGB(dartTail, 0, 0, 0);
        buffer.setRGB(dartHead, dartColorRGB[0] / reciprocalOfBrightness, dartColorRGB[1] / reciprocalOfBrightness, dartColorRGB[2] / reciprocalOfBrightness);

        ledStrip.setData(buffer);
        
    }

    public void setAll(int r, int g, int b) {
        for (int i = 0; i < LedCount; i++ ) {
            buffer.setRGB(i, r / reciprocalOfBrightness,g / reciprocalOfBrightness, b / reciprocalOfBrightness);
        }
        
        ledStrip.setData(buffer);
    }

    public void cycleColor() {
        switch (currentColor) {
            case Purple:
                setAll(255, 255, 0);
                currentColor = Color.Yellow;
                break;
            case Yellow:
                setAll(0, 0, 0);
                // setAll(255, 0, 0);
                currentColor = Color.Off;
                break;
            case Off:
                setAll(255, 0, 255);
                currentColor = Color.Purple;
                break;
        }
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
        //m_candle.animate(a);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}