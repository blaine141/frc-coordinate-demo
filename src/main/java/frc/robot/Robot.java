// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private final Joystick m_controller = new Joystick(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // Field to display robot location
  private final Field2d m_field = new Field2d();

  private final AprilTagFieldLayout fieldLayout;

  public Robot() {
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      System.err.println(e);
      throw new RuntimeException(e);
    }
  }

  @Override
  public void robotInit() {
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Display robot location
    Pose2d botWRTAlliance = m_swerve.getRobotPose();
    Pose2d botPose = botWRTAlliance;

    // Pose2d mapWRTAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red
    //     ? new Pose2d(new Translation2d(16.542, 8.014), new Rotation2d(3.14159))
    //     : new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    // botPose = botWRTAlliance.relativeTo(mapWRTAlliance);
    
    m_field.setRobotPose(botPose);


    // // Display april tag locations
    // for (AprilTag tag : fieldLayout.getTags()) {
    //   var tagPose = tag.pose.toPose2d();
    //   var fieldObject = m_field.getObject("Tag " + tag.ID);
    //   fieldObject.setPose(tagPose);
    // }

    Rotation3d test = new Rotation3d(1, .5, .3);
    double roll = test.getX();
    double pitch = test.getY();
    double yaw = test.getZ();

  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get chassis speeds from joystick inputs.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(1), 0.02))
            * Drivetrain.kMaxSpeed;

    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(0), 0.02))
            * Drivetrain.kMaxSpeed;

    final var rot =
        m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(2), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    // Move the robot with joystick inputs
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
