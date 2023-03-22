// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.ObjectInputFilter.Status;

import com.fasterxml.jackson.databind.introspect.DefaultAccessorNamingStrategy.FirstCharBasedValidator;
import com.fasterxml.jackson.databind.node.DoubleNode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double        ROBOT_MASS   = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double        CHASSIS_MASS = ROBOT_MASS;
  public static final Translation3d CHASSIS_CG   = new Translation3d(0, 0, Units.inchesToMeters(8));
  public static final double        LOOP_TIME    = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig xAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig yAutoPID     = new PIDFConfig(0.7, 0, 0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0.0, 0.01);

    public static final double MAX_SPEED        = 4;
    public static final double MAX_ACCELERATION = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class Intake {
    public static final double wheelTopSpeedCube = 0.1;
    public static final double wheelBottomSpeedCube = 0.1;
    public static final double wheelTopSpeedCone = 0.3;
    public static final double wheelBottomSpeedCone = 0.3;
    public static final int topMotor = 28;
    public static final int bottomMotor = 27;
    public static final int extendChannel = 12;
    public static final int retractChannel = 11;
    public static final boolean reverseSolenoid = false;
    public static final double wheelBottomSpeedEject = 0.45;
    public static final double wheelTopSpeedEject = 0.45;
  }

  public static class Grabber {
    public static final int rightMotorId = 24;
    public static final int leftMotorId = 25;
    public static final int extendChannel = 9;
    public static final int retractChannel = 8;
    public static final double offCurrent = -1;
    public static final boolean disableMotors = false;
    public static final double motorSpeeds = 0.5;
  }

  public static class Arm {
    public static final float revToMetersConversionFactor = (float) 1; // need to set
    public static final int extendSparkMaxId = 22;
    public static final float revToAngleConversionFactor = 360/112;
    public static final int rotateSparkMaxId = 23;
    public static final double iExtension = 0;
    public static final double pExtension = 0.06505600363016129*2;  
    public static final double dExtension = 0;
    public static final double pRotating = 0.1/2*0.95;
    public static final double iRotating = 0;
    public static final double dRotating = 0;
    public static final int extendCancoderid = -1;
    public static final int rotateCancoderid = -1;
    public static final float extendOffset = 0;
    public static final float rotateOffset = 0;
    public static final double maxSpeedExtend = 0.1;
    public static final double maxSpeedRotate = 0.1;
    public static final double maxAccelExtend = 0.1;
    public static final double maxAccelRotate = 0.1;
    public static final int rotateFollowSparkMaxId = 26;
    public static final int rollingAverageLength = 20;

    public static final ArmFeedforward[] feedForwardMap = {
      new ArmFeedforward(0, 0, 0),
      new ArmFeedforward(0, 0, 0),
      new ArmFeedforward(0, 0, 0),
      new ArmFeedforward(0, 0, 0),
      new ArmFeedforward(0, 0, 0),
      new ArmFeedforward(0, 0, 0),
    };
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;

    public static final float MAX_INPUT_CHANGE = 0.05f;

  }
}
