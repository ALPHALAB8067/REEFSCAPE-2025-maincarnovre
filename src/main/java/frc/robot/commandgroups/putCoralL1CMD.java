// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.goToL1;
import frc.robot.commands.claw.ClawExitCMD;
import frc.robot.commands.claw.ClawGoStraight;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;

public class putCoralL1CMD extends SequentialCommandGroup {

  private final ClawSS clawss;
  private final ARM_SS armss;

  public putCoralL1CMD(ClawSS pClawSS, ARM_SS pArm_SS) {
    
    clawss = pClawSS;
    armss = pArm_SS;

    addCommands(

    new ParallelCommandGroup(
      new ClawGoStraight(clawss),
      new goToL1(armss)
    ),

    new ClawExitCMD(clawss).withTimeout(1)

    );
  }
}
