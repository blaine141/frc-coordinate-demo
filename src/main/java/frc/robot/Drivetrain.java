// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });

  // Simulation classes
  private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  public Pose2d getRobotPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // Get updated pose info from SmartDashboard
    double x = SmartDashboard.getNumber("RobotWRTAlliance X", 0);
    double y = SmartDashboard.getNumber("RobotWRTAlliance Y", 0);
    double angle = SmartDashboard.getNumber("RobotWRTAlliance Angle", 0);

    // Update our odometry if the user has changed our pose in SmartDashboard
    if (x != getRobotPose().getTranslation().getX() || y != getRobotPose().getTranslation().getY()
        || angle != getRobotPose().getRotation().getDegrees()) {

      // Reset our gyro to the new angle
      m_gyroSim.setAngle(-angle);

      // Reset our odometry to the new pose
      m_odometry.resetPosition(m_gyro.getRotation2d(), new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      }, new Pose2d(x, y, m_gyro.getRotation2d()));
    }

    // Update our odometry with robot movement
    updateOdometry();

    // Update the SmartDashboard with our current pose
    Pose2d botWRTField = getRobotPose();
    Translation2d translation = botWRTField.getTranslation();
    Rotation2d rotation = botWRTField.getRotation();
    SmartDashboard.putNumber("RobotWRTAlliance X", translation.getX());
    SmartDashboard.putNumber("RobotWRTAlliance Y", translation.getY());
    SmartDashboard.putNumber("RobotWRTAlliance Angle", rotation.getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // Perform simulation on SwerveModules
    m_backLeft.updateSimulation();
    m_backRight.updateSimulation();
    m_frontLeft.updateSimulation();
    m_frontRight.updateSimulation();

    // Get robot velocity from SwerveModules
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());

    // Update gyro angle and rate based on robot velocity
    // Clockwise is positive for gyro.
    m_gyroSim.setRate(-Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));
    m_gyroSim.setAngle(m_gyro.getAngle() + m_gyro.getRate() * 0.02);

  }
}
