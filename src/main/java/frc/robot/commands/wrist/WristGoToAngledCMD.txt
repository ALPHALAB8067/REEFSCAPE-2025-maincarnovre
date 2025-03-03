package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSS;

public class WristGoToAngledCMD extends Command {

  private final WristSS wristSS;

  public WristGoToAngledCMD(WristSS pWristSS) {

    wristSS = pWristSS;

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    wristSS.goToAngled();
  }

  @Override
  public void end(boolean interrupted) {
    wristSS.stopWrist();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
