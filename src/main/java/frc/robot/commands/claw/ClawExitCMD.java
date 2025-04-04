package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSS;

public class ClawExitCMD extends Command {

  private final ClawSS clawss;

  public ClawExitCMD(ClawSS clawSS) {
      
    clawss = clawSS;
  
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    clawss.reverseWheel();
  }

  @Override
  public void end(boolean interrupted) {
    clawss.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
