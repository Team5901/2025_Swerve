// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

      // Controller and joystick
  private final Joystick joystick = new Joystick(0);
  private final XboxController controller_2 = new XboxController(1);
  private final int translationAxis = Joystick.AxisType.kY.value;
  private final int strafeAxis = Joystick.AxisType.kX.value;
  private final int rotationAxis = Joystick.AxisType.kZ.value;

  private final JoystickButton zeroGyro = new JoystickButton(joystick, 11);
    private final Trigger moveArm = new Trigger(() -> Math.abs(controller_2.getLeftY()) > 0.1);
    private final Trigger moveElevator = new Trigger(() -> Math.abs(controller_2.getRightY()) > 0.1);

    private final JoystickButton Level1A =
      new JoystickButton(controller_2, XboxController.Button.kA.value);

    private final JoystickButton Level2X =
      new JoystickButton(controller_2, XboxController.Button.kX.value);

    private final JoystickButton IntakeRollersIn =
      new JoystickButton(controller_2, XboxController.Button.kLeftBumper.value);
    private final JoystickButton IntakeRollersOut =
      new JoystickButton(controller_2, XboxController.Button.kRightBumper.value);

    PositionTracker positionTracker = new PositionTracker();
    private Mechanism2d mechanisms = new Mechanism2d(5, 3);
    private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.25);
    private MechanismLigament2d armLigament = root
            .append(new MechanismLigament2d("armLigament", Units.inchesToMeters(10), 270,
                    5,
                    new Color8Bit(Color.kRed)));
    private final Supplier<Pose3d> carriagePoseSupplier = new Supplier<Pose3d>() {

        @Override
        public Pose3d get() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'get'");
        }
        
    };

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    TalonFX _talonArm = new TalonFX(12);
    TalonFX _talonSpool = new TalonFX(10);
    //public final Arm arm = new Arm(positionTracker, armLigament, carriagePoseSupplier);
    Intake intake = new Intake();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getRawAxis(translationAxis) * (joystick.getRawButton(1) ? 1d : 0.71d) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getRawAxis(strafeAxis) * (joystick.getRawButton(1) ? 1d : 0.71d) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRawAxis(rotationAxis) * (joystick.getRawButton(1) ? 1d : 0.25d) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        moveArm.whileTrue(new InstantCommand(() -> _talonArm.setControl(new DutyCycleOut(0.5 * controller_2.getLeftY()))));
        moveArm.onFalse(new InstantCommand(() -> _talonArm.setControl(new DutyCycleOut(-0.01))));
        moveElevator.whileTrue(new InstantCommand(() -> _talonSpool.setControl(new DutyCycleOut(0.5 * controller_2.getRightY()))));
        moveElevator.onFalse(new InstantCommand(() -> _talonSpool.setControl(new DutyCycleOut(-0.01))));
        //Level1A.onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1, arm));
        //Level2X.onTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2, arm));

        IntakeRollersIn.whileTrue(new InstantCommand(() -> intake.setRollerVoltage(-3), intake));
        IntakeRollersIn.onFalse(new InstantCommand(() -> intake.setRollerVoltage(0), intake));
        IntakeRollersOut.whileTrue(new InstantCommand(() -> intake.setRollerVoltage(3), intake));
        //IntakeRollersOut.onFalse(new InstantCommand(() -> intake.setRollerVoltage(0), intake));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        zeroGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
