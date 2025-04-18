package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetPositionElevatorCommand extends Command {
    private Elevator elevator;
    private double setPoint;

    /** Creates a new SetPosition. */
    public SetPositionElevatorCommand(Elevator elevator, double setPoint) {
        this.elevator = elevator;
        this.setPoint = setPoint;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.setPosition(setPoint);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return elevator.atPosition();
    }
}
