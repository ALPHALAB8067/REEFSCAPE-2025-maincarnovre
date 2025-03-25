package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmSlightlyUp_CMD;
import frc.robot.commands.dontbreakintake;
import frc.robot.commands.dontbreakintakedown;
import frc.robot.commands.goToInt;
import frc.robot.commands.claw.ClawGoAngledCMD;
import frc.robot.commands.claw.ClawGoStraight;
import frc.robot.commands.claw.ClawIntakeCMD;
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
      new dontbreakintake(armss),
      new ClawGoAngledCMD(wrist),
      new intakeDownCMD(intake),
      new WaitCommand(2),
      new goToInt(pArm_SS),
      //arm takes coral
      
      new ParallelCommandGroup(
        new ClawIntakeCMD(pClawSS),
        new intakeWheelCMD(pIntakeSS)
//        new WristIntakeCMD(wrist).until(wrist.hasSomething() == true),
        )
        

      


      


    );
  }
}
