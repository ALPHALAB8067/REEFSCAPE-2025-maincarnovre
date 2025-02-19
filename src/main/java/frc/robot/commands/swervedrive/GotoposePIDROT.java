// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GotoposePIDROT extends Command {
  public final SwerveSubsystem mSwerveSubsystem; 
  double X;
  double Y;
  double Z;
  PIDController TranslationPIDX = new PIDController(1,0.00,0);
  PIDController TranslationPIDY = new PIDController(0.5,0.00,0) ;
  PIDController TurnPID = new PIDController(0.15,0,0) ;
    double XPoseValue;
    double YPoseValue;
    double RotPoseValue;
    


  /** Creates a new GotoposePID. */
  public GotoposePIDROT(SwerveSubsystem mSwerveSubsystem, double mX, double mY, double mRot) {
    this.mSwerveSubsystem = mSwerveSubsystem;
    X = mX;
    Y = mY;
    Z = mRot;
   

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.mSwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double XPose = mSwerveSubsystem.getPose().getX();
      double XPoseValue = TranslationPIDX.calculate(XPose, X);
        double YPose = mSwerveSubsystem.getPose().getY();
        double YPoseValue = TranslationPIDY.calculate(YPose, Y);
          double RotationPose = mSwerveSubsystem.getPose().getRotation().getRadians();
          double RotPoseValue = TurnPID.calculate(RotationPose, Math.toRadians(Z));


    
  mSwerveSubsystem.drive(new ChassisSpeeds(XPoseValue, YPoseValue, RotPoseValue));

   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
