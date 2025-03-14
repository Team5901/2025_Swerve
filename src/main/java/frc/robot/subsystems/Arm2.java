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
import static frc.robot.Constants.Arm;

@LoggedObject
public class Arm2 extends SubsystemBase implements BaseIntake {
    @Log
    private final SparkMax motor;

    private SparkMaxConfig motorConfig;

    public Arm2() {
        motorConfig = new SparkMaxConfig();
        motorConfig
                .inverted(Arm.MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Arm.CURRENT_LIMIT);
                //.encoder.countsPerRevolution(42);

        motor = new SparkMax(Arm.MOTOR_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setArmVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public Command runRollersCommand() {
        return Commands.startEnd(
                () -> setArmVoltage(3),
                () -> setArmVoltage(0))
                .withName("intake.runRollers");
    }

    @Override
    public Command reverseRollersCommand() {
        return Commands.startEnd(
                () -> setArmVoltage(-12),
                () -> setArmVoltage(0))
                .withName("intake.reverseRollers");
    }
}
