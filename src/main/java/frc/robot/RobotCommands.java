package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.subsystems.Arm;

public class RobotCommands {
    public static ScoreLevel lastScore = ScoreLevel.None;

    public static Command prepareCoralScoreCommand(ScoreLevel level, Arm arm) {
        //ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        switch (level) {
            case L1 -> {
                //elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                //elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                //elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                //elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ScoreLevel");
            }
        }

        return Commands.runOnce(() -> {
            lastScore = level;
        })
                .andThen(Commands.parallel(
                        arm.moveToPositionCommand(() -> armPosition).asProxy(),
                        Commands.waitSeconds(0.5)));
                                //.andThen(elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())));
    }
}
