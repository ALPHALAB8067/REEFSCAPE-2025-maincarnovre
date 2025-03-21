package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.dontbreakintake;
import frc.robot.commands.goToRest;
import frc.robot.commands.intake.intakeDownCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.IntakeSS;


public class IntakeInCMD extends SequentialCommandGroup {

  private final IntakeSS intake;
  private final ARM_SS armss;

  public IntakeInCMD(IntakeSS pIntakeSS, ARM_SS pArm_SS) {
    
    intake = pIntakeSS;
    armss = pArm_SS;

    addCommands(

      new dontbreakintake(armss),
      new intakeDownCMD(intake),
      new goToRest(armss)


    );
  }
}
