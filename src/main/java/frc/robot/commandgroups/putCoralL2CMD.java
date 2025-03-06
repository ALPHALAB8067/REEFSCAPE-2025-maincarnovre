// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.goToL2;
import frc.robot.commands.claw.ClawExitCMD;
import frc.robot.commands.claw.ClawGoStraight;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;

public class putCoralL2CMD extends SequentialCommandGroup {

  private final ClawSS clawSS;
  private final ARM_SS armss;

  public putCoralL2CMD(ClawSS pClawSS, ARM_SS pArm_SS) {
    
    clawSS = pClawSS;
    armss = pArm_SS;

    addCommands(

    new ParallelCommandGroup(
      new ClawGoStraight(clawSS),
      new goToL2(armss)
    ),

    new ClawExitCMD(clawSS).withTimeout(1)

    );
  }
}
