package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;

public class AutoScoreCoral extends Command {
  private Superstructure superstructure;
  private boolean resetElevator = false;

  // Add reset elevator parameter for G-side L3 auto to stop it from puncturing algae
  public AutoScoreCoral(Superstructure superstructure, boolean resetElevator) {
    this.superstructure = superstructure;
    this.resetElevator = resetElevator;
  }

  public AutoScoreCoral(Superstructure superstructure) {
    this(superstructure, true);
  }

  @Override
  public void initialize() {
    superstructure.requestScore();
  }

  @Override
  public boolean isFinished() {
    return !superstructure.pieceSecured();
  }

  @Override
  public void end(boolean intterupted) {
    if (resetElevator) {
      superstructure.requestIdle();
    }
  }
}
