// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;


public class LightSubsystem extends SubsystemBase
{

    private final CANdle m_candle = new CANdle(3, "rio");
    private final int LedCount = 300;

    private enum Color {
        Yellow,
        Purple,
        Rainbow
    }

    private Color color = Color.Yellow;

    public void playAuto() {
        m_candle.animate(new RainbowAnimation());
    }
    
    public void cycleColor() {
        switch (color) {
            case Yellow:
                color = Color.Purple;
                m_candle.animate(new StrobeAnimation(255, 0, 255));
                //m_candle.setLEDs(255,0, 255);
                break;
            case Purple:
                color = Color.Rainbow;
                m_candle.animate(new RainbowAnimation());
                //m_candle.setLEDs(255,255, 0);
                break;
            case Rainbow:
                color = Color.Yellow;
                m_candle.animate(new StrobeAnimation(255, 255, 0));
                //m_candle.setLEDs(255,255, 0);
                break;
                    }
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}

