// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Adding comment to delete later

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.Arm2;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SetPositionElevatorCommand;

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
    private final Trigger moveElevator = new Trigger(() -> Math.abs(controller_2.getLeftY()) > 0.1);
    private final Trigger moveArm = new Trigger(() -> Math.abs(controller_2.getRightY()) > 0.1);
    

    private final JoystickButton Level4A =
      new JoystickButton(controller_2, XboxController.Button.kA.value);

    private final JoystickButton HomeB =
      new JoystickButton(controller_2, XboxController.Button.kB.value);

    private final JoystickButton RestRight =
      new JoystickButton(controller_2, XboxController.Button.kRightBumper.value);

    private final JoystickButton Level3Left =
      new JoystickButton(controller_2, XboxController.Button.kLeftBumper.value);

    private final JoystickButton ClimbDownX =
      new JoystickButton(controller_2, XboxController.Button.kX.value);

    private final JoystickButton ClimbUpY = new JoystickButton(controller_2, XboxController.Button.kY.value);

    //private final JoystickButton IntakeRollersIn =
     // new JoystickButton(controller_2, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton IntakeRollersOut =
      //new JoystickButton(controller_2, XboxController.Button.kRightBumper.value);

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
    TalonFX _talonClimb = new TalonFX(3);
    private final MotionMagicVoltage arm_mmReq = new MotionMagicVoltage(0);
    TalonFX _talonSpool = new TalonFX(10);
    public final Arm2 arm = new Arm2();
    public final Elevator elevator = new Elevator("elevator");
    Intake intake = new Intake();
    final VoltageOut m_request = new VoltageOut(0);

    /* Commands */
    private final SetPositionElevatorCommand moveElevatorToL3 = new SetPositionElevatorCommand(elevator, -11);

    private final SetPositionElevatorCommand moveElevatorToRest = new SetPositionElevatorCommand(elevator, -8);

    private final SetPositionElevatorCommand moveElevatorToL4 = new SetPositionElevatorCommand(elevator, -38);

    private final SetPositionElevatorCommand moveElevatorToHome = new SetPositionElevatorCommand(elevator, 0);
 
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private TalonFXConfiguration cfg = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    /* Configure gear ratio */
    private FeedbackConfigs fdb = cfg.Feedback;

    /* Configure Motion Magic */
    //private final MotionMagicConfigs mm = cfg.MotionMagic;
    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

   //private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

    Slot0Configs slot0 = cfg.Slot0;

    StatusCode status = StatusCode.StatusCodeNotInitialized;


    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
              drivetrain.applyRequest(() ->
                  drive.withVelocityX(-joystick.getRawAxis(translationAxis) * (joystick.getRawButton(1) ? 0.24d : 0.71d) * MaxSpeed) // Drive forward with negative Y (forward)
                      .withVelocityY(-joystick.getRawAxis(strafeAxis) * (joystick.getRawButton(1) ? 0.24d : 0.71d) * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(-joystick.getRawAxis(rotationAxis) * (joystick.getRawButton(1) ? 0.12d : 0.25d) * MaxAngularRate) // Drive counterclockwise with negative X (left)
              )
        );

        //fdb.SensorToMechanismRatio = 25; // 25 rotor rotations per mechanism rotation
        //mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.5)) // 5 (mechanism) rotations per second cruise
          //  .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)) // Take approximately 0.5 seconds to reach max vel
            // Take approximately 0.1 seconds to reach max accel 
            //.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        slot0.kS = 0.4; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.1; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP =0; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0; // A velocity error of 1 rps results in 0.5 V output
        _talonClimb.getConfigurator().apply(cfg);

        ClimbUpY.whileTrue(new InstantCommand(() -> _talonClimb.setControl(m_velocityVoltage.withVelocity((75)))));
        ClimbUpY.onFalse(new InstantCommand(() -> _talonClimb.setControl(m_velocityVoltage.withVelocity((0)))));
        ClimbDownX.whileTrue(new InstantCommand(() -> _talonClimb.setControl(m_velocityVoltage.withVelocity((-75)))));
        ClimbDownX.onFalse(new InstantCommand(() -> _talonClimb.setControl(m_velocityVoltage.withVelocity((0)))));
        //moveArm.whileTrue(new InstantCommand(() -> _talonArm.setControl(m_request.withOutput(12.0 * controller_2.getLeftY()))));
        //Level1A.onTrue(new InstantCommand(() -> _talonArm.setControl(m_positionVoltage.withPosition(1))));
        moveArm.whileTrue(new InstantCommand(() -> arm.setArmVoltage(2.5 * Math.signum(controller_2.getRightY()))));
        moveArm.onFalse(new InstantCommand(() -> arm.setArmVoltage(0)));
        moveElevator.whileTrue(new InstantCommand(() -> elevator.setElevatorVoltage(4 * Math.signum(controller_2.getLeftY()))));
        moveElevator.onFalse(new InstantCommand(() -> elevator.setElevatorVoltage(0)));
        Level4A.onTrue(moveElevatorToL4);
        HomeB.onTrue(moveElevatorToHome);
        Level3Left.onTrue(moveElevatorToL3);
        RestRight.onTrue(moveElevatorToRest);
        

        //IntakeRollersIn.whileTrue(new InstantCommand(() -> intake.setRollerVoltage(-3), intake));
        //IntakeRollersIn.onFalse(new InstantCommand(() -> intake.setRollerVoltage(0), intake));
        //IntakeRollersOut.whileTrue(new InstantCommand(() -> intake.setRollerVoltage(3), intake));
        //IntakeRollersOut.onFalse(new InstantCommand(() -> intake.setRollerVoltage(0), intake));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        zeroGyro.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

       /* creeperMode.whileTrue( 
          drivetrain.applyRequest(() ->
                drive2.withVelocityX(-joystick.getRawAxis(translationAxis) * (joystick.getRawButton(1) ? 1d : 0.71d) * (MaxSpeed/2)) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getRawAxis(strafeAxis) * (joystick.getRawButton(1) ? 1d : 0.71d) * (MaxSpeed/2)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRawAxis(rotationAxis) * (joystick.getRawButton(1) ? 1d : 0.25d) * (MaxAngularRate/2)) // Drive counterclockwise with negative X (left)
            ));*/

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
      /* Run the path selected from the auto chooser */
      return autoChooser.getSelected();
  }

  public void slowSpeed() {
    MaxSpeed = MaxSpeed/2;
  }

  public void regSpeed() {
    MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }
}
