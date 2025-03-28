package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSS;

public class ClawGoAngledCMD extends Command {

  private final ClawSS clawss;

  public ClawGoAngledCMD(ClawSS clawSS) {

    clawss = clawSS;

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    clawss.goToAngled(90);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    
    return clawss.IsAtAngle(90);
    
  }
}
