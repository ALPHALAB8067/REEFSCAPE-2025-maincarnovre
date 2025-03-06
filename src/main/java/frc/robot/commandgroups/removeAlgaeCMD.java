package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;

public class removeAlgaeCMD extends SequentialCommandGroup {

  private final ClawSS clawSS;
  private final ARM_SS armSS;

  public removeAlgaeCMD(ClawSS pClawSS, ARM_SS pArm_SS) {
    
    clawSS = pClawSS;
    armSS = pArm_SS;

    addCommands(

      /*new //bringarmtoalgae(armSS)
      .andThen(new WristExitCMD(wristSS).withTimeout(5)
      )
      */


    );

  }
}