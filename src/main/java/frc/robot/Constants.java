package frc.robot;

import java.util.List;

//import com.pathplanner.lib.config.ModuleConfig;
//import com.pathplanner.lib.config.RobotConfig;
//import com.techhounds.houndutil.houndlib.leds.BaseLEDSection;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
//import frc.robot.subsystems.LEDs.LEDState;

public class Constants {

    public static final class Arm {
        public static enum ArmPosition {
            BOTTOM(-Math.PI / 2.0 + Units.degreesToRadians(5)),
            HORIZONTAL(0),
            L1(0),
            L2(Units.degreesToRadians(25)),
            //L2(Units.degreesToRadians(55)), // reef angle
            L3(Units.degreesToRadians(55)),
            L4(1.033),
            TOP(Math.PI / 2.0);

            public final double value;

            private ArmPosition(double value) {
                this.value = value;
            }
        }

        public static final double MOTION_LIMIT = -0.7;
        public static final double SCORING_MOVEMENT = -0.8;

        public static final int MOTOR_ID = 12;
        public static final boolean MOTOR_INVERTED = false;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 25.0; // TODO
        public static final double MASS_KG = Units.lbsToKilograms(10); // TODO
        public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
        public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
        public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

        public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
        public static final double MAX_ANGLE_RADIANS = Math.PI / 2.0;

        public static final int CURRENT_LIMIT = 50;

        public static final double kP = 10; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
        public static final double kS = 0.017964; // TODO
        public static final double kG = 0.321192; // TODO
        public static final double kV = 0.876084;// TODO
        public static final double kA = 0.206676;// TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 8; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class Elevator {
        public static enum ElevatorPosition {
            BOTTOM(0.0698),
            INTAKE_PREP(0.55),
            INTAKE(0.355),
            ALGAE_L2(0.884),
            ALGAE_L3(1.234),

            L1(0.323),
            L2(0.31),
            L3(0.70),
            L4(1.27),
            TOP(1.57);

            public final double value;

            private ElevatorPosition(double value) {
                this.value = value;
            }
        }

        public static final double MOTION_LIMIT = 0.3;

        public static final double SCORING_MOVEMENT = -0.25;

        public static final int MOTOR_ID = 4;
        public static final boolean MOTOR_INVERTED = false;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 5.0;
        public static final double MASS_KG = Units.lbsToKilograms(20);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.32) / 2.0; // TODO
        public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0.005; // TODO
        public static final double MAX_HEIGHT_METERS = 1.57; // TODO

        public static final int CURRENT_LIMIT = 60;

        public static final double kP = 50; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 5; // TODO
        public static final double kS = 0.095388; // TODO
        public static final double kG = 0.54402; // TODO
        public static final double kV = 7.43; // TODO
        public static final double kA = 1.0; // TODO
        public static final double TOLERANCE = 0.5;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.3; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class Intake {
        public static final int MOTOR_ID = 11;
        public static final boolean MOTOR_INVERTED = true;
        public static final int CURRENT_LIMIT = 60;
    }
}
