package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.techhounds.houndutil.houndlib.subsystems.BaseIntake;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Elevator.*;

@LoggedObject
public class Elevator extends SubsystemBase implements BaseIntake {
    @Log
    private final SparkMax motor;
    private SparkClosedLoopController pidController;
    private double setPoint;
    private double encoderTolerance;
    private String name;
    private SparkMaxConfig motorConfig;

    public Elevator(String name) {
        this.encoderTolerance = TOLERANCE;
        this.name = name;
        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        pidController = motor.getClosedLoopController();
        motorConfig = new SparkMaxConfig();
        motorConfig
                .inverted(MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIMIT)
                .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kP, kI, kD);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the motor to a target position.
     * @param position The desired position in encoder units
     */
    public void setPosition(double position) {
        setPoint = position;
        pidController.setReference(position, ControlType.kPosition);
    }

    /**
     * Checks if the motor is within the set position tolerance.
     * @return True if within tolerance, false otherwise
     */
    public boolean atPosition() {
        double currentPos = motor.getEncoder().getPosition();
        return (currentPos >= setPoint - encoderTolerance) && (currentPos <= setPoint + encoderTolerance);
    }
    
    public void setElevatorVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(name + "_SetPoint", setPoint);
        SmartDashboard.putNumber(name + "_Pose", motor.getEncoder().getPosition());
        SmartDashboard.putBoolean(name + "_AtPose", atPosition());
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
