// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PositionsDictionnary;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.PositionType_SS;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class dontbreakintakecomeback extends Command {
  /** Creates a new dontbreakintake. */
  private final ARM_SS mArm_SS;
  PositionType_SS mCurrent;
  double startpos;



  public dontbreakintakecomeback(ARM_SS pArm_SS) {
    // Use addRequirements() here to declare subsystem dependencies.
    mArm_SS = pArm_SS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startpos = mArm_SS.GetPositionTypeAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mCurrent = mArm_SS.whereAmI();
    if(mCurrent == PositionsDictionnary.mPositionRest) {
      mArm_SS.setArmPosition(mArm_SS.GetPositionTypeAngle() + dontbreakintakedown.degree3);
    }
    if(mCurrent == PositionsDictionnary.mPositionL1) {
      mArm_SS.setArmPosition(mArm_SS.GetPositionTypeAngle() + dontbreakintakedown.degree3);
    }
    if(mCurrent == PositionsDictionnary.mPositionL2) {
      mArm_SS.setArmPosition(mArm_SS.GetPositionTypeAngle() + dontbreakintakedown.degree3);
    }
    if(mCurrent == PositionsDictionnary.mPositionIntake) {
      mArm_SS.setArmPosition(mArm_SS.GetPositionTypeAngle() + dontbreakintakedown.degree3);
    }
    

  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    //startpos == mArm_SS.GetPositionTypeAngle() - dontbreakintakedown.degree
    if(mArm_SS.isArmInPosition(mArm_SS.GetPositionTypeAngle() + dontbreakintakedown.degree2, 2)) {
      return true;
    } else {
      return false;
    }
    
  }
}
