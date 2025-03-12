package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmSlightlyUp_CMD;
import frc.robot.commands.claw.ClawGoAngledCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;
import frc.robot.subsystems.IntakeSS;
import frc.robot.commands.intake.intakeDownCMD;
import frc.robot.commands.intake.intakeWheelCMD;

public class IntakeOutCMD extends SequentialCommandGroup {

  private final IntakeSS intake;
  private final ARM_SS armss;
  private final ClawSS wrist;
      
  public IntakeOutCMD(IntakeSS pIntakeSS, ARM_SS pArm_SS, ClawSS pClawSS) {
    
    intake = pIntakeSS;
    armss = pArm_SS;
    wrist = pClawSS;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new ArmSlightlyUp_CMD(armss),
      new ClawGoAngledCMD(wrist),
      
      new intakeDownCMD(intake),
      //arm takes coral

      
      new ParallelCommandGroup(
//        new WristIntakeCMD(wrist).until(wrist.hasSomething() == true),
        )
        

      


      


    );
  }
}
