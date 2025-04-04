// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.claw.ClawExitCMD;
import frc.robot.commands.intake.intakeWheelReverseCMD;
import frc.robot.subsystems.ClawSS;
import frc.robot.subsystems.IntakeSS;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OuttakeCMDG extends ParallelCommandGroup {
  ClawSS mClawSS;
  IntakeSS mIntakeSS;
  /** Creates a new OuttakeCMDG. */
  public OuttakeCMDG(ClawSS pClawSS, IntakeSS pIntakeSS) {
    mIntakeSS = pIntakeSS;
    mClawSS = pClawSS;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClawExitCMD(mClawSS),
      new intakeWheelReverseCMD(mIntakeSS)
    );
  }
}
