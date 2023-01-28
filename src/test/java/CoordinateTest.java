import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class CoordinateTest {

  private final Pose2d m_botWRTField = new Pose2d(new Translation2d(2.629, 2.088), new Rotation2d(3.05));
  private final Transform2d m_botToCamera = new Transform2d(new Translation2d(0.1, -0.3), new Rotation2d(0.3));

  private final AprilTagFieldLayout fieldLayout;

  public CoordinateTest() {
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }


  @Test // marks this method as a test
  void AprilTagProblem() {
    // Prepare the test by solving for the transform from the camera to the tag
    Pose2d tagWRTField = fieldLayout.getTags().get(7).pose.toPose2d();
    Transform2d cameraToTag = tagWRTField.minus(m_botWRTField.plus(m_botToCamera));
    Transform2d botToCamera = m_botToCamera;

    Pose2d botWRTField = new Pose2d();

    // Solve for the robot pose using only the following:
    // 1. cameraToTag, The transform from the camera to the tag
    // 2. tagWRTField, The pose of the tag
    // 3. botToCamera, The transform from the robot to the camera
    // Place the result in the variable botWRTField

    // TODO: Solve for botWRTField

    // Start Solution

    // End Solution    

    // Test if correct =
    assertEquals(
        m_botWRTField, botWRTField);
  }

}