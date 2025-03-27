// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.goToCoralStation;
import frc.robot.commands.claw.ClawGoAngledCMD;
import frc.robot.commands.claw.ClawIntakeCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetStationCoralCMDGteleop extends SequentialCommandGroup {
  /** Creates a new GetStationCoralCMDG. */
  private final ARM_SS mArm_SS;
  private final ClawSS mClawSS;
  private final ClawSS wrist;


  public GetStationCoralCMDGteleop(ARM_SS pArm_SS, ClawSS pClawSS){
    mArm_SS = pArm_SS;
    mClawSS = pClawSS;
    wrist = pClawSS;


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
new ParallelCommandGroup(
    new goToCoralStation(mArm_SS),
    new ClawGoAngledCMD(wrist),
    new ClawIntakeCMD(pClawSS)
)

    );
  }
}
