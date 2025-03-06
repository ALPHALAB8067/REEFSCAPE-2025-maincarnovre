package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.claw.ClawGoAngledCMD;
import frc.robot.commands.goToRest;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;

public class closeRobotCMD extends ParallelCommandGroup {

  private final ARM_SS armSS;
  private final ClawSS clawSS;

  public closeRobotCMD(ARM_SS pArm_SS, ClawSS cplawSS) {
    
    armSS = pArm_SS;
    clawSS = cplawSS;

    addCommands(

      new goToRest(armSS),
      new ClawGoAngledCMD(clawSS)

    );
  }
}
