package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.dontbreakintake;
import frc.robot.commands.claw.ClawGoAngledCMD;
import frc.robot.commands.claw.ClawIntakeCMD;
import frc.robot.commands.intake.intakeDownCMD;
import frc.robot.commands.intake.intakeUpCMD;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.ClawSS;
import frc.robot.subsystems.IntakeSS;

//sequential bc we can't make all the commands at once. 
//we can set parallel command groups in a sequence
public class getCoralCMD extends SequentialCommandGroup {

  private final IntakeSS intake;
  private final ClawSS wrist;
  private final ARM_SS armSS;

  public getCoralCMD(IntakeSS pIntakeSS, ClawSS pWristss, ARM_SS pArm_SS) {

    intake = pIntakeSS;
    wrist = pWristss;
    armSS = pArm_SS;

    addCommands(
      
    new dontbreakintake(armSS),

    new ParallelCommandGroup(
    new frc.robot.commands.intake.intakeWheelCMD(intake).withTimeout(3),
      new intakeDownCMD(intake),
      new ClawGoAngledCMD(wrist)
    ),

    new ParallelCommandGroup(
      //bring arm to upped intake
    ),

      new ClawIntakeCMD(pWristss).withTimeout(3)
    );

    
  }
}
