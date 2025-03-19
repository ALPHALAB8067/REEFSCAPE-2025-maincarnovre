package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PositionsDictionnary;
import frc.robot.subsystems.ARM_SS;
import frc.robot.subsystems.IntakeSS;

public class intakeUpCMD extends Command {

  private final IntakeSS intakeSS; 
  private final ARM_SS arm_SS;
  private final double ArmMinimumThreshold = 0; 

  public intakeUpCMD(IntakeSS pIntakeSS, ARM_SS pArmSS) {

    intakeSS = pIntakeSS;
    arm_SS = pArmSS;
    

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSS.intakeUp();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
