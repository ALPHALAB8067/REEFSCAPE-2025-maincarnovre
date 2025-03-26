package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.dontbreakintake2;
import frc.robot.commands.dontbreakWrist;
import frc.robot.commands.goToRest;
import frc.robot.commands.claw.RetainCoralCMD;
import frc.robot.commands.intake.intakeUpCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;
import frc.robot.subsystems.IntakeSS;


public class CloseRobotCMDG extends SequentialCommandGroup {

  private final IntakeSS intake;
  private final ARM_SS armss;
  private final ClawSS clawSS;

  public CloseRobotCMDG(IntakeSS pIntakeSS, ARM_SS pArm_SS, ClawSS pClawSS) {
    
    intake = pIntakeSS;
    armss = pArm_SS;
    clawSS = pClawSS;


    addCommands(
    new ParallelRaceGroup(
      new RetainCoralCMD(clawSS),
      new SequentialCommandGroup(
          new dontbreakWrist(armss),
          new dontbreakintake2(armss),
          new intakeUpCMD(intake),
          new WaitCommand(0.5),
           new goToRest(armss)
        )
      )
      

     // new intakeUpCMD(intake)


    );
  }
}
