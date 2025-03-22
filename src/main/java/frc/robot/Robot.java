// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Date;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private PhotonCamera camera;

  public Robot() {
    m_robotContainer = new RobotContainer();
    camera = new PhotonCamera("front_camera");
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
    double targetRange = 0.0;
    int targetID = 0;
    int resultCount = 0;
    int targetCount = 0;
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    resultCount = results.size();
    if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        targetCount = result.getTargets().size();
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() > 0) {
                    targetID = target.getFiducialId();
                    // Found Tag, record its information
                    targetYaw = target.getYaw();
                    targetRange =
                            PhotonUtils.calculateDistanceToTargetMeters(
                                    1, // Measured with a tape measure, or in CAD.
                                    1, // From 2024 game manual for ID
                                    Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                    Units.degreesToRadians(target.getPitch()));

                    targetVisible = true;
                }
            }
        }
    }

    // Put debug information to the dashboard
    SmartDashboard.putNumber("Vision Result Count", resultCount);
    SmartDashboard.putNumber("Vision Target Count", targetCount);
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
    SmartDashboard.putNumber("Vision Target Range (m)", targetRange);
    SmartDashboard.putNumber("Vision Target Yaw", targetYaw);
    SmartDashboard.putNumber("Target ID", targetID);
    SmartDashboard.putNumber("Time", new Date().getTime());
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
