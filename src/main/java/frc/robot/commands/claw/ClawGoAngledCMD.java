package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSS;
import frc.robot.util.WristConstants;

public class ClawGoAngledCMD extends Command {

  private final ClawSS clawss;

  public ClawGoAngledCMD(ClawSS clawSS) {

    clawss = clawSS;

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    clawss.goToAngled();
  }

  @Override
  public void end(boolean interrupted) {
    clawss.stopWrist();
  }

  @Override
  public boolean isFinished() {
    
    if(clawss.getRotatePosition() == WristConstants.angledPosition) {
      return true;
    } else 
      return false;
    
  }
}
