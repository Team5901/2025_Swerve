// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Date;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private PhotonCamera camera;

  private PhotonPoseEstimator photonPoseEstimator;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  Optional<EstimatedRobotPose> robotPose;

  public Robot() {
    m_robotContainer = new RobotContainer();
    camera = new PhotonCamera("front_camera");
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetPitch = 0.0;
    double targetRange = 0.0;
    int targetID = 0;
    Optional<Pose3d> targetPose;
    double targetHeight = 0.0;
    double targetX = 0.0;
    double targetY = 0.0;
    int resultCount = 0;
    int targetCount = 0;
    Translation2d targetTranslation;

    double poseTarget = 0;
    double poseX = 0;
    double poseY = 0;
    double poseHeading = 0;

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    resultCount = results.size();
    if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        targetCount = result.getTargets().size();
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera

            // Pose Estimation not working yet
            robotPose = photonPoseEstimator.update(result);
            if (robotPose.isPresent()) {
                poseTarget = robotPose.get().targetsUsed.get(0).fiducialId;
                poseX = robotPose.get().estimatedPose.getTranslation().getX();
                poseY = robotPose.get().estimatedPose.getTranslation().getY();
                poseHeading = Units.radiansToDegrees(robotPose.get().estimatedPose.getRotation().getAngle());
            }
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (target.getFiducialId() > 0) {
                    targetID = target.getFiducialId();
                    targetPose = aprilTagFieldLayout.getTagPose(targetID);
                    if(targetPose.isPresent()) {
                        targetHeight = targetPose.get().getTranslation().getZ();
                        targetX = targetPose.get().getTranslation().getX();
                        targetY = targetPose.get().getTranslation().getY();
                    }
                    // Found Tag, record its information
                    targetYaw = Units.degreesToRadians(target.getYaw());
                    targetPitch = Units.degreesToRadians(target.getPitch());
                    // IMPORTANT: Won't work if target and camera are exact same height
                    targetRange =
                            PhotonUtils.calculateDistanceToTargetMeters(
                                    1, // Measured with a tape measure, or in CAD.
                                    targetHeight, // From 2024 game manual for ID
                                    Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
                                    Units.degreesToRadians(target.getPitch()));
                    targetVisible = true;
                    targetTranslation = PhotonUtils.estimateCameraToTargetTranslation(targetRange, Rotation2d.fromDegrees(-target.getYaw()));
                }
            }
        }
    }

    // Put debug information to the dashboard
    SmartDashboard.putNumber("Time", new Date().getTime());
    SmartDashboard.putBoolean("Camera Connected", camera.isConnected());
    SmartDashboard.putNumber("Result Count", (double) resultCount);
    SmartDashboard.putNumber("Target Count", (double) targetCount);
    SmartDashboard.putBoolean("Target Visible", targetVisible);
    SmartDashboard.putNumber("Target ID", targetID);
    SmartDashboard.putNumber("Target Range (m)", targetRange);
    SmartDashboard.putNumber("Target Yaw", targetYaw);
    SmartDashboard.putNumber("Target Pitch", targetPitch);

    // Target Position
    SmartDashboard.putNumber("Target Height", targetHeight);
    SmartDashboard.putNumber("Target X", targetX);
    SmartDashboard.putNumber("Target Y", targetY);

    // Pose Estimation not working yet
    SmartDashboard.putNumber("Target used for pose", poseTarget);
    SmartDashboard.putNumber("Pose X", poseX);
    SmartDashboard.putNumber("Pose Y", poseY);
    SmartDashboard.putNumber("Pose Heading", poseHeading);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
