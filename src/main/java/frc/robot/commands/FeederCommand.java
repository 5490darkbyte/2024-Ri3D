package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.shooter.Shooter;
public class FeederCommand extends Command {
    Shooter shooter;

    public FeederCommand() {
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //this was originally 6 for left and 0 for right?
        shooter.setPowers(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //this was originally 6 for left and 0 for right?
        shooter.setPowers(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
