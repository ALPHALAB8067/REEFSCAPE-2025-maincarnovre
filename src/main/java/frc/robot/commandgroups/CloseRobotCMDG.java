package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.dontbreakIntake2;
import frc.robot.commands.dontbreakWrist;
import frc.robot.commands.dontbreakintake;
import frc.robot.commands.goToRest;
import frc.robot.commands.intake.intakeUpCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.IntakeSS;


public class CloseRobotCMDG extends SequentialCommandGroup {

  private final IntakeSS intake;
  private final ARM_SS armss;

  public CloseRobotCMDG(IntakeSS pIntakeSS, ARM_SS pArm_SS) {
    
    intake = pIntakeSS;
    armss = pArm_SS;

    addCommands(

      new dontbreakWrist(armss),
      new dontbreakIntake2(armss),
      new intakeUpCMD(intake),
      new WaitCommand(0.3),
      new goToRest(armss)

     // new intakeUpCMD(intake)


    );
  }
}
