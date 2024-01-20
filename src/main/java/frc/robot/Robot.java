// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  // Key poses
  private final Pose2d m_goalWRTField = new Pose2d(16.54, 5.55, Rotation2d.fromDegrees(180));
  private final Transform3d m_botToAngledCamera = new Transform3d(0, 0, .5, new Rotation3d(0, -Math.PI/4, 0));
  private final Transform3d m_botToFrontCamera = new Transform3d(.5, 0, .5, new Rotation3d(0, 0, 0));

  private final AprilTagFieldLayout fieldLayout;

  StructPublisher<Pose2d> botWRTAlliancePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/botWRTAlliance", Pose2d.struct).publish();
  StructArrayPublisher<Pose3d> tag3dPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SmartDashboard/3D/AprilTags", Pose3d.struct).publish();
  StructPublisher<Transform2d> robotToGoalPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/2D/robotToGoal", Transform2d.struct).publish();
  StructPublisher<Transform2d> goalToRobotPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/2D/goalToRobot", Transform2d.struct).publish();
  StructPublisher<Transform2d> robotToTag1Publisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/2D/robotToTag1", Transform2d.struct).publish();
  StructPublisher<Transform2d> tag1ToRobotPublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/2D/tag1ToRobot", Transform2d.struct).publish();
  StructPublisher<Transform3d> angledCameraToTag1Publisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/3D/angledCameraToTag1", Transform3d.struct).publish();
  StructPublisher<Transform3d> frontCameraToTag1Publisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/3D/frontCameraToTag1", Transform3d.struct).publish();
  StructPublisher<Pose3d> angledCameraWRTAlliancePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/3D/angledCameraWRTAlliance", Pose3d.struct).publish();
      StructPublisher<Pose3d> frontCameraWRTAlliancePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/SmartDashboard/3D/frontCameraWRTAlliance", Pose3d.struct).publish();

  public Robot() {
    // Load the April Tag field layout from the resource file
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
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

    // Prepare field / alliance conversion for later
    Pose2d fieldWRTAlliance = fieldWRTAlliance();
    Pose2d allianceWRTField = allianceWRTField();
    Pose3d allianceWRTField3d = pose2dToPose3d(allianceWRTField);

    // Display robot location
    Pose2d botWRTAlliance = m_swerve.getRobotPose();    // << Look here
    Pose2d botWRTField = botWRTAlliance.relativeTo(fieldWRTAlliance);
    botWRTAlliancePublisher.set(botWRTAlliance);
    m_field.setRobotPose(botWRTAlliance);

    // Get Tag 1 pose
    Pose2d tag1WRTField = fieldLayout.getTags().get(0).pose.toPose2d();

    // Get important transforms
    Transform2d robotToGoal = m_goalWRTField.minus(botWRTField);
    Transform2d robotToTag1 = tag1WRTField.minus(botWRTField);
    Transform2d tag1ToRobot = robotToTag1.inverse();        // << Look here
    Transform2d goalToRobot = robotToGoal.inverse();

    // Publish transforms
    robotToGoalPublisher.set(robotToGoal);
    goalToRobotPublisher.set(goalToRobot);
    robotToTag1Publisher.set(robotToTag1);
    tag1ToRobotPublisher.set(tag1ToRobot);


    // Push April Tag positions to Smart Dashboard
    // AdvantageScope expects things in the alliance frame so we need to do some conversion. 
    var poseList = new ArrayList<Pose3d>();
    for (AprilTag tag : fieldLayout.getTags()) {
      var tagWRTField = tag.pose;
      var tagWRTAlliance = tagWRTField.relativeTo(allianceWRTField3d);
      poseList.add(tagWRTAlliance);
    }
    tag3dPublisher.set(poseList.toArray(new Pose3d[0]));

    // Add field objects to display in the odometry tab
    // AdvantageScope expects things in the alliance frame so we need to do some conversion.
    var goalWRTAlliance = m_goalWRTField.relativeTo(allianceWRTField);
    m_field.getObject("Goal").setPose(goalWRTAlliance);
    for (AprilTag tag : fieldLayout.getTags()) {
      var tagWRTField = tag.pose;
      var tagWRTAlliance = tagWRTField.relativeTo(allianceWRTField3d);
      m_field.getObject("Tag " + tag.ID).setPose(tagWRTAlliance.toPose2d());
    }


    // Get the 3D pose of the robot
    Pose3d botWRTField3d = pose2dToPose3d(botWRTField);
    Pose3d botWRTAlliance3d = pose2dToPose3d(botWRTAlliance);

    // Get the 3D pose of tag 1
    Pose3d tag1WRTField3d = fieldLayout.getTags().get(0).pose;

    // Get the 3D pose of the camera
    Pose3d angledCameraWRTAlliance = botWRTAlliance3d.transformBy(m_botToAngledCamera);
    Pose3d angledCameraWRTField = botWRTField3d.transformBy(m_botToAngledCamera);       // << Look here
    Pose3d frontCameraWRTAlliance = botWRTAlliance3d.transformBy(m_botToFrontCamera);
    Pose3d frontCameraWRTField = botWRTField3d.transformBy(m_botToFrontCamera);

    // Get the interesting transforms
    Transform3d angledCameraToTag1 = tag1WRTField3d.minus(angledCameraWRTField);
    Transform3d frontCameratoTag1 = tag1WRTField3d.minus(frontCameraWRTField);

    // Publish to Smart Dashboard
    angledCameraToTag1Publisher.set(angledCameraToTag1);
    frontCameraToTag1Publisher.set(frontCameratoTag1);
    angledCameraWRTAlliancePublisher.set(angledCameraWRTAlliance);
    frontCameraWRTAlliancePublisher.set(frontCameraWRTAlliance);
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private static Pose2d fieldWRTAlliance() {
    var alliance = DriverStation.getAlliance();
    if (!alliance.isEmpty() && alliance.get() == Alliance.Red)
      return new Pose2d(16.54, 8.21, Rotation2d.fromDegrees(180));
    else
      return new Pose2d();
  }

  private static Pose2d allianceWRTField() {
    // This is a weird special case where fiel to alliance is the same as alliance to field
    return fieldWRTAlliance();
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get chassis speeds from joystick inputs.
    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(1), 0.02))
        * Drivetrain.kMaxSpeed;

    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(0), 0.02))
        * Drivetrain.kMaxSpeed;

    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRawAxis(2), 0.02))
        * Drivetrain.kMaxAngularSpeed;

    // Move the robot with joystick inputs
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  private static Pose3d pose2dToPose3d(Pose2d pose2d) {
    return new Pose3d(pose2d.getX(), pose2d.getY(), 0,
        new Rotation3d(0, 0, pose2d.getRotation().getRadians()));
  }
}
