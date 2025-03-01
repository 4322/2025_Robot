package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class AutoFeedCoral extends Command {
  private Superstructure superstructure;

  public AutoFeedCoral(Superstructure superstructure) {
    this.superstructure = superstructure;
  }

  @Override
  public void initialize() {
    superstructure.requestFeed();
  }

  @Override
  public boolean isFinished() {
    return superstructure.pieceSecured();
  }

  @Override
  public void end(boolean intterupted) {
    superstructure.requestIdle();
  }
}
