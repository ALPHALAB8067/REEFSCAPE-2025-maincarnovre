// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.goToL4;
import frc.robot.commands.scoreWrist;
import frc.robot.commands.claw.ClawGoStraight;
import frc.robot.commands.claw.RetainCoralCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;

public class L4_CMDG extends SequentialCommandGroup {

  private final ClawSS clawSS;
  private final ARM_SS armss;

  public L4_CMDG(ClawSS pClawSS, ARM_SS pArm_SS) {
    
    clawSS = pClawSS;
    armss = pArm_SS;

    addCommands(
  new ParallelRaceGroup(
    new RetainCoralCMD(pClawSS),
    new ParallelCommandGroup(
      new ClawGoStraight(clawSS),
      new goToL4(armss)
      )
    )
    );
  }
}
