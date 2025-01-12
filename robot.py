
#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from math import pi
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import drivetrain
import ntcore
from wpimath.geometry._geometry import Pose2d, Pose3d, Transform2d, Transform3d, Rotation2d, Rotation3d

# Key poses
algaeWRTAlliance = Pose2d(1.2, 4.04, Rotation2d())
botToAngledCamera = Transform3d(0, 0, .5, Rotation3d(0, -pi/4, 0))
botToFrontCamera = Transform3d(.5, 0, .5, Rotation3d(0, 0, 0))

fieldLayout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)
numTags = len(fieldLayout.getTags())


botWRTAlliancePublisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/botWRTAlliance", Pose2d).publish()
tag3dPublisher = ntcore.NetworkTableInstance.getDefault().getStructArrayTopic("/SmartDashboard/3D/AprilTags", Pose3d).publish()
robotToalgaePublisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/2D/robotToalgae", Transform2d).publish()
algaeToRobotPublisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/2D/algaeToRobot", Transform2d).publish()
robotToTag1Publisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/2D/robotToTag1", Transform2d).publish()
tag1ToRobotPublisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/2D/tag1ToRobot", Transform2d).publish()
tag1WRTAlliancePublisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/2D/tag1WRTAlliance", Pose2d).publish()
algaeWRTAlliancePublisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/2D/algaeWRTAlliance", Pose2d).publish()
angledCameraToTag1Publisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/3D/angledCameraToTag1", Transform3d).publish()
frontCameraToTag1Publisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/3D/frontCameraToTag1", Transform3d).publish()
angledCameraWRTAlliancePublisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/3D/angledCameraWRTAlliance", Pose3d).publish()
frontCameraWRTAlliancePublisher = ntcore.NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/3D/frontCameraWRTAlliance", Pose3d).publish()


aprilTagRedOrigin = Pose3d(1.01, -.15, 0, Rotation3d())
aprilTagBlueOrigin = Pose3d(18.558, 7.902, 0, Rotation3d(0, 0, 180))

def pose2dToPose3d(pose2d: Pose2d) -> Pose3d:
    return Pose3d(pose2d.x, pose2d.y, 0, Rotation3d(0, 0, pose2d.rotation().radians()))
  


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()

        # Slew rate limiters to make joystick inputs more gentle 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.deadband = 0.1
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

    def autonomousPeriodic(self) -> None:
        if self.isSimulation():
            self.simulationPeriodic()
                  
        self.driveWithJoystick(False)
        


    def teleopPeriodic(self) -> None:
        if self.isSimulation():
            self.simulationPeriodic()
                        
        self.driveWithJoystick(True)
        self.swerve.updateOdometry()

        fieldLayout.setOrigin(fieldLayout.OriginPosition.kBlueAllianceWallRightSide if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue else fieldLayout.OriginPosition.kRedAllianceWallRightSide)





        # Display robot location
        botWRTAlliance = self.swerve.odometry.getPose()   # << Look here
        botWRTAlliancePublisher.set(botWRTAlliance)





        # Get Tag 1 pose
        tag1WRTAlliance = fieldLayout.getTagPose(1).toPose2d()

        # Get important transforms
        robotToalgae = algaeWRTAlliance - botWRTAlliance
        robotToTag1 = tag1WRTAlliance - botWRTAlliance
        tag1ToRobot = robotToTag1.inverse()        # << Look here
        algaeToRobot = robotToalgae.inverse()

        # Publish transforms
        robotToalgaePublisher.set(robotToalgae)
        algaeToRobotPublisher.set(algaeToRobot)
        robotToTag1Publisher.set(robotToTag1)
        tag1ToRobotPublisher.set(tag1ToRobot)
        tag1WRTAlliancePublisher.set(tag1WRTAlliance)
        algaeWRTAlliancePublisher.set(algaeWRTAlliance)





        # Push April Tag positions to Smart Dashboard
        # AdvantageScope expects things in the alliance frame so we need to do some conversion. 
        poseList = []
        for tagNum in range(1, numTags+1):
            tagWRTAlliance = fieldLayout.getTagPose(tagNum)
            poseList.append(tagWRTAlliance)
        
        tag3dPublisher.set(poseList)

        # Get the 3D pose of the robot
        botWRTAlliance3d = pose2dToPose3d(botWRTAlliance)

        # Get the 3D pose of tag 1
        tag1WRTAlliance3d = fieldLayout.getTagPose(1)

        # Get the 3D pose of the camera
        angledCameraWRTAlliance = botWRTAlliance3d + botToAngledCamera   # << Look here
        frontCameraWRTAlliance = botWRTAlliance3d + botToFrontCamera

        # Get the interesting transforms
        angledCameraToTag1 = tag1WRTAlliance3d - angledCameraWRTAlliance
        frontCameratoTag1 = tag1WRTAlliance3d - frontCameraWRTAlliance

        # Publish to Smart Dashboard
        angledCameraToTag1Publisher.set(angledCameraToTag1)
        frontCameraToTag1Publisher.set(frontCameratoTag1)
        angledCameraWRTAlliancePublisher.set(angledCameraWRTAlliance)
        frontCameraWRTAlliancePublisher.set(frontCameraWRTAlliance)
        botWRTAlliancePublisher.set(self.swerve.odometry.getPose())
        

    def simulationPeriodic(self) -> None:
        self.swerve.updateSimulation(self.getPeriod())
		

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), self.deadband)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), self.deadband)
            )
            * drivetrain.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), self.deadband)
            )
            * drivetrain.kMaxSpeed
        )


        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

    