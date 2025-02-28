package frc.robot;

public class Constants {
    public static final class ArmSubsystem {
        public static final int deviceID = 14;
        public static final int deviceID2 = 15;
        public static final boolean isInverted = true;
        public static final boolean followerInverted = true;
    
        // FeedForward Control
        public static final double ks = 0.00;
        public static final double kv = 1; // 0.2
        public static final double kg = 0.5; // 0.75
    
        public static final double kP = 0.05;
        public static final double kI = 0.00;
        public static final double kD = 0.0;
        public static final double kA = 0.0;
        public static final double kIz = 0;
        public static final double kFF = 0.0001;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;
    
        public static final double gearRatio = 50 * (58 / 12); // Arm chain loop
        // public static final double sprocketDiameterInch = 1.92;
    
        // Arm details
        public static final double maxVelocityDegreesPerSec = 270.0;
        public static final double maxAccelerationDegreesPerSec = 360.0;
        public static final double armVolts = 6.0;
        // public static final double armVelocityRPM = (1 / 2) * 60;
    
        // motor shaft details
        public static final int maxCurrentAmps = 30;
        public static final double maxAngularVelocityRPM = (maxVelocityDegreesPerSec / 6) * gearRatio;
        public static final double maxAngularAccRPMPerSec =
            (maxAccelerationDegreesPerSec / 6) * gearRatio;
        public static final double minOutputVelocityRPM = 10.0; // requests below this no voltage output
        public static final double allowableSmartMotionPosErrorRotations = 3.0 * gearRatio;
        public static final double autoPositionErrorInch = 2.0;
    
        // Degrees
        public static final double armSoftLimitLowerAngle = -15;
        public static final double armPosOut = -10;
        public static final double armPosSpeaker = 18;
        public static final double armPosFarSpeaker = 47.5;
        public static final double armYeetIt = 70;
    
        public static final double armPosAmp = 108.0;
        public static final double armPosTrap = 75.0;
        public static final double armPosIn = 10;
        public static final double armSoftLimitUpperAngle = 120.0;
        public static final double goalTolerance = 1;
        public static final double allowableTeleopErrorInch = 1.0;
      }
}
