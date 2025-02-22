// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class FieldConstants
{
  

   public static Pose2d PoteauL = new Pose2d(3.706,5.060,Rotation2d.fromDegrees(-60));
   public static Pose2d PoteauK = new Pose2d(3.992,5.249,Rotation2d.fromDegrees(-60));
   public static Pose2d PoteauJ = new Pose2d(4.989,5.249,Rotation2d.fromDegrees(-120));


}
