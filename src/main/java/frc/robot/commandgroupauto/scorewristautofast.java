// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroupauto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.scoreWristauto;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scorewristautofast extends ParallelCommandGroup {
  
  private final ARM_SS armss;
  private final ClawSS clawSS;
  /** Creates a new scorewristautofast. */
  public scorewristautofast(ARM_SS pArm_SS, ClawSS pClawSS) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    armss = pArm_SS;
    clawSS = pClawSS;
    addCommands(
      new scoreWristauto(armss, clawSS).withTimeout(0.5)
    );
  }
}
