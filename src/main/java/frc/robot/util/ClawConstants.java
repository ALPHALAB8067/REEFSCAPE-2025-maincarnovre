// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ClawConstants {
  
  public static final int rotateSparkMaxPort = 18;

  public static final boolean rotateIsInverted = true;
  public static final double positionConversionFactor = 3.60;
  public static final IdleMode rotateIdleMode = IdleMode.kCoast;
  public static final FeedbackSensor feedbacksensor = FeedbackSensor.kPrimaryEncoder;
  public static final double pValue = 0.05;
  public static final double iValue = 0;
  public static final double dValue = 0;

  public static final int wheelSparkMaxPort = 19;
  public static final boolean wheelIsInverted = false;
  public static final IdleMode wheelIdleMode = IdleMode.kCoast;
  public static final double straightPosition = 0.0;
  public static final double angledPosition = 90.0;
  public static final double turnWheelPercent = 1;
  public static final double reverseWheelPercent = -0.8;
  public static final double retainCoralPercent = 0.25;
}
