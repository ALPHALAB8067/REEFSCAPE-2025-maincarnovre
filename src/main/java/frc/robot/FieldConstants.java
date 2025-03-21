// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

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
   public final class RedAlliancePoses{

      public static Pose2d PoteauAR = new Pose2d(14.363, 3.862, Rotation2d.fromDegrees(180));
      public static Pose2d PoteauBR = new Pose2d(14.363, 4.200, Rotation2d.fromDegrees(180));
      public static Pose2d PoteauCR = new Pose2d(13.846, 5.045, Rotation2d.fromDegrees(-120));
      public static Pose2d PoteauDR = new Pose2d(13.559, 5.234, Rotation2d.fromDegrees(-120));
      public static Pose2d PoteauER = new Pose2d(12.546, 5.226, Rotation2d.fromDegrees(-60));
      public static Pose2d PoteauFR = new Pose2d(12.262, 5.067, Rotation2d.fromDegrees(-60.000));
      public static Pose2d PoteauGR = new Pose2d(11.762, 4.195, Rotation2d.fromDegrees(0));
      public static Pose2d PoteauHR = new Pose2d(11.762, 3.850, Rotation2d.fromDegrees(0));
      public static Pose2d PoteauIR = new Pose2d(12.264, 2.979, Rotation2d.fromDegrees(60));
      public static Pose2d PoteauJR = new Pose2d(12.562, 2.823, Rotation2d.fromDegrees(60));
      public static Pose2d PoteauKR = new Pose2d(13.573, 2.811, Rotation2d.fromDegrees(120));
      public static Pose2d PoteauLR = new Pose2d(13.865, 2.980, Rotation2d.fromDegrees(120));
   }

   public final class BlueAlliancePoses{

      public static Pose2d PoteauA = new Pose2d(3.181, 4.177, Rotation2d.fromDegrees(0));
      public static Pose2d PoteauB = new Pose2d(3.181, 3.849, Rotation2d.fromDegrees(0));
      public static Pose2d PoteauC = new Pose2d(3.676, 2.972, Rotation2d.fromDegrees(60));
      public static Pose2d PoteauD = new Pose2d(4.000, 2.816, Rotation2d.fromDegrees(60));
      public static Pose2d PoteauE = new Pose2d(4.982, 2.835, Rotation2d.fromDegrees(120));
      public static Pose2d PoteauF = new Pose2d(5.245, 3.000, Rotation2d.fromDegrees(120));
      public static Pose2d PoteauG = new Pose2d(5.762, 3.849, Rotation2d.fromDegrees(180));
      public static Pose2d PoteauH = new Pose2d(5.772, 4.181, Rotation2d.fromDegrees(180));
      public static Pose2d PoteauI = new Pose2d(5.255, 5.039, Rotation2d.fromDegrees(-120));
      public static Pose2d PoteauJ = new Pose2d(4.989,5.249,Rotation2d.fromDegrees(-120));
      public static Pose2d PoteauK = new Pose2d(3.992,5.249,Rotation2d.fromDegrees(-60));
      public static Pose2d PoteauL = new Pose2d(3.706,5.060,Rotation2d.fromDegrees(-60));
   }
   public final class StationRed {
      public static Pose2d RedDown = new Pose2d(15.980, 0.759, Rotation2d.fromDegrees(-60));
   }

}

