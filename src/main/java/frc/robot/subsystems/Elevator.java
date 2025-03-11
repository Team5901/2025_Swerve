package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Elevator.*;

@LoggedObject
public class Elevator extends SubsystemBase implements BaseIntake {
    @Log
    private final SparkMax motor;

    private SparkMaxConfig motorConfig;

    public Elevator() {
        motorConfig = new SparkMaxConfig();
        motorConfig
                .inverted(MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIMIT);

        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setElevatorVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public Command runRollersCommand() {
        return Commands.startEnd(
                () -> setElevatorVoltage(3),
                () -> setElevatorVoltage(0))
                .withName("intake.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return Commands.startEnd(
                () -> setElevatorVoltage(-12),
                () -> setElevatorVoltage(0))
                .withName("intake.reverseRollers");
    }
}
