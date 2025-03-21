package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmSlightlyUp_CMD;
import frc.robot.commands.dontbreakintake;
import frc.robot.commands.dontbreakintakedown;
import frc.robot.commands.claw.ClawGoAngledCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;
import frc.robot.subsystems.IntakeSS;

public class IntakeOutCMD extends SequentialCommandGroup {
    
  private final IntakeSS intakess;
  private final ARM_SS arm_SS;
  private final ClawSS clawss;

  public IntakeOutCMD(IntakeSS pIntakeSS, ARM_SS pArm_SS, ClawSS pClawSS) {
    
    intakess = pIntakeSS;
    arm_SS = pArm_SS;
    clawss = pClawSS;
    
    
    addCommands(

      new dontbreakintake(arm_SS),
      new IntakeOutCMD(intakess, arm_SS, clawss),
      new dontbreakintakedown(arm_SS),

      new ParallelCommandGroup(
        new

      )
      


    );
  }
}
