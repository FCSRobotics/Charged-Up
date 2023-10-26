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
package frc.robot.subsystems

import com.ctre.phoenix.led.Animation
import com.ctre.phoenix.led.CANdle
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.*

class LightSubsystem : SubsystemBase() {
    private val m_candle = CANdle(3, "rio")
    private val LedCount = 32
    private val buffer = AddressableLEDBuffer(LedCount)
    private val m_toAnimate: Animation? = null


    // public enum Color {
    //     Yellow,
    //     Purple,
    //     Off
    // }
    // private boolean overridenByBoost = false;
    //private Color currentColor = Color.Yellow;
    var a: Animation? = null
    private val ledStrip = AddressableLED(0)
    private val reciprocalOfBrightness = 2
    private var rainbowFirstPixelHue = 0
    private var tally = 0
    private var lastAlternate = System.currentTimeMillis()
    private var color = intArrayOf(0, 255 / reciprocalOfBrightness, 0)
    private var hue = 104
    fun setColor(r: Int, g: Int, b: Int, h: Int) {
        hue = h
        if (color[0] != r || color[1] != g || color[2] != b) {
            color = intArrayOf(r / reciprocalOfBrightness, g / reciprocalOfBrightness, b / reciprocalOfBrightness)
            for (i in 0 until evenBuffer.length) {
                if (i % 2 == 0) {
                    evenBuffer.setRGB(i, color[0], color[1], color[2])
                } else {
                    oddBuffer.setRGB(i, color[0], color[1], color[2])
                }
            }
        }
    }

    fun fade() {
        val time = System.currentTimeMillis()
        val brightness = Math.abs(Math.sin((time / 5000).toDouble())).toFloat()
        for (i in 0 until buffer.length) {
            buffer.setRGB(i, Math.round(color[0].toFloat() * brightness), Math.round(color[1].toFloat() * brightness), Math.round(color[2].toFloat() * brightness))
        }
        // if(!overridenByBoost) {
        ledStrip.setData(buffer)
        // }
    }

    fun rainbow() {
        tally++
        for (i in 0 until buffer.length) {
            val hue = (rainbowFirstPixelHue + i * 180 / buffer.length) % 180
            buffer.setHSV(i, hue, 255, 255 / reciprocalOfBrightness)
        }
        rainbowFirstPixelHue += 3
        rainbowFirstPixelHue %= 180
        ledStrip.setData(buffer)
    }

    fun rainbow2() {
        tally++
        for (i in 0 until buffer.length) {
            val hue = (rainbowFirstPixelHue + i * 180 / buffer.length) % 180
            buffer.setHSV(i, 255, 255, hue / reciprocalOfBrightness)
        }
        rainbowFirstPixelHue += 3
        rainbowFirstPixelHue %= 180
        ledStrip.setData(buffer)
    }

    private var alternateEven = false
    private val evenBuffer = AddressableLEDBuffer(LedCount)
    private val oddBuffer = AddressableLEDBuffer(evenBuffer.length)
    fun alternate() {
        tally++
        if (System.currentTimeMillis() - lastAlternate >= 500) {
            alternateEven = !alternateEven
            lastAlternate = System.currentTimeMillis()
            if (alternateEven) {
                ledStrip.setData(evenBuffer)
            } else {
                ledStrip.setData(oddBuffer)
            }
        }
    }

    private val movingUpLed = 0
    private var movingDownLed = 0
    private val turningOnLeds = false
    private var lastUpdate: Long = 0
    private var ripplePoint = 0
    private var lastRipple: Long = 0
    fun rain() {
        if (System.currentTimeMillis() - lastUpdate > 100) {
            lastUpdate = System.currentTimeMillis()
            if (movingDownLed > 0) {
                buffer.setRGB(movingDownLed, color[0] / reciprocalOfBrightness, color[1] / reciprocalOfBrightness, color[2] / reciprocalOfBrightness)
            } else if (movingDownLed == 0) {
                movingDownLed = ripplePoint
                for (i in 0 until LedCount) {
                    buffer.setRGB(i, 0, 0, 0)
                }
                lastRipple = System.currentTimeMillis()
                ripplePoint = Random().nextInt(buffer.length)
            }
            movingDownLed += 2
            ledStrip.setData(buffer)
        }
    }

    private val dartSize = 7
    private var dartTail = LedCount - dartSize - 1
    private var dartHead = 0
    private val dartColorRGB = intArrayOf(255, 255, 255)

    init {
        ledStrip.setLength(buffer.length)
        setAll(0, 255 / reciprocalOfBrightness, 0)
        ledStrip.start()
        for (i in 0 until evenBuffer.length) {
            if (i % 2 == 0) {
                evenBuffer.setRGB(i, color[0], color[1], color[2])
            } else {
                oddBuffer.setRGB(i, color[0], color[1], color[2])
            }
        }
    }

    fun dart() {
        tally++
        dartTail++
        dartHead++
        dartTail %= LedCount - 1
        dartHead %= LedCount - 1
        buffer.setRGB(dartTail, 0, 0, 0)
        buffer.setRGB(dartHead, color[0], color[1], color[2])
        ledStrip.setData(buffer)
    }

    private fun setAll(r: Int, g: Int, b: Int) {
        for (i in 0 until LedCount) {
            buffer.setRGB(i, r / reciprocalOfBrightness, g / reciprocalOfBrightness, b / reciprocalOfBrightness)
        }
        ledStrip.setData(buffer)
    }

    fun cycleColor() {
        // switch (currentColor) {
        //     case Purple:
        //         setAll(255, 255, 0);
        //         currentColor = Color.Yellow;
        //         break;
        //     case Yellow:
        //         setAll(0, 0, 0);
        //         // setAll(255, 0, 0);
        //         currentColor = Color.Off;
        //         break;
        //     case Off:
        //         setAll(255, 0, 255);
        //         currentColor = Color.Purple;
        //         break;
        // }
    }

    override fun periodic() {
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

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    fun alert() {
        this.setAll(0, 255, 0)
    }
}