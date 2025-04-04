package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSS;

public class ClawGoStraight extends Command {

  private final ClawSS clawss;

  public ClawGoStraight(ClawSS clawSS) {

    clawss = clawSS;

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    clawss.goToStraight(0);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {

    return false;

  }
}
