package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;

public class ManualScore extends Command {
    private Superstructure superstructure;

    public ManualScore(Superstructure superstructure) {
        this.superstructure = superstructure;
        addRequirements(superstructure);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (RobotContainer.operatorBoard.getFlipRequested()) {
            superstructure.requestPreScoreFlip();
        }
        else {
            superstructure.requestPreScore();
        }

        if (RobotContainer.driver.getRightTriggerAxis() > 0.5) {
            superstructure.requestScore();
        }
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.requestIdle();
    }
}