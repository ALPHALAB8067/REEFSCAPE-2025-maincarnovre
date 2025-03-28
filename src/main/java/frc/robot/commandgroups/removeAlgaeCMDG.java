package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.goToAlgae;
import frc.robot.commands.goToL1;
import frc.robot.commands.scoreWrist;
import frc.robot.commands.claw.ClawGoAngledCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;

public class removeAlgaeCMDG extends SequentialCommandGroup {

  private final ClawSS clawSS;
  private final ARM_SS armSS;

  public removeAlgaeCMDG(ClawSS pClawSS, ARM_SS pArm_SS) {
    
    clawSS = pClawSS;
    armSS = pArm_SS;

    addCommands(
      new ParallelCommandGroup(
        new ClawGoAngledCMD(pClawSS),
        new goToAlgae(pArm_SS)
      )
  
      


    );

  }
}