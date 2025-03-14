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

import frc.robot.PositionTracker;
import frc.robot.Constants.Arm.ArmPosition;

@LoggedObject
public class Arm2 extends SubsystemBase implements BaseIntake {
    @Log
    private final SparkMax motor;

    private SparkMaxConfig motorConfig;

    public Arm2(PositionTracker positionTracker) {
        motorConfig = new SparkMaxConfig();
        motorConfig
                .inverted(Arm.MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(Arm.CURRENT_LIMIT);

        motor = new SparkMax(Arm.MOTOR_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Position
        motorConfig.encoder.setPositionConversionFactor(Arm.ENCODER_POSITION_CONVERSION_FACTOR);
        motorConfig.encoder.setVelocityConversionFactor(Arm.ENCODER_VELOCITY_CONVERSION_FACTOR);
        this.positionTracker = positionTracker;
        positionTracker.setArmAngleSupplier(this::getPosition);
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

    public Command moveToPositionCommand(Supplier<Double> positionSupplier) {
    return new InstantCommand(() -> {
        double positionDegrees = positionSupplier.get();
        double positionRadians = Math.toRadians(positionDegrees);
        // Logic to move the arm to the specified position in radians
        // This might involve setting the motor to a specific position using PID control
        // For example:
        motor.getEncoder().setPosition(positionRadians);
    });
}
}
