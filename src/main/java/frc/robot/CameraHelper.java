// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot;

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.swervelib.SwerveDrivetrain;

/** Helper for using a camera */
public class CameraHelper
{
  private final AprilTagFieldLayout tags;
  private final PhotonCamera camera;
  private final Transform3d robotToCam;
  private final PhotonPoseEstimator estimator;
  private final NetworkTableEntry nt_flag;
  private int successes = 0;

  /** @param tags Field info
   *  @param camera_name Camera name ("front") in photonvision network tablee ntries
   *  @param status_name Name used to show status on dashboard
   *  @param pos_x Camera pos. relative to center of robot, X
   *  @param pos_y Y
   *  @param pos_z Z
   *  @param heading Camera heading (yaw)
   *  @param pitch Camera down/up tilt
   */
  public CameraHelper(AprilTagFieldLayout tags, String camera_name,
                      String status_name,
                      double pos_x, double pos_y, double pos_z,
                      double heading,
                      double pitch)
  {
    this.tags = tags;
    camera = new PhotonCamera(camera_name);

    nt_flag = SmartDashboard.getEntry(status_name);

    // TODO: Allow access to the camera from a computer when tethered to the USB port on the roboRIO
    // PortForwarder.add(5800, "photonvision.local", 5800);

    // Where is the camera mounted relative to the center of the robot?
    // Example: mounted facing forward, 30cm forward of center, 10cm up from floor.
    robotToCam = new Transform3d(new Translation3d(pos_x, pos_y, pos_z),
                                 new Rotation3d(0,
                                                Math.toRadians(pitch),
                                                Math.toRadians(heading)));

    // Prepare estimator
    // Which strategy?
    estimator = new PhotonPoseEstimator(tags, PoseStrategy.AVERAGE_BEST_TARGETS, robotToCam);
  }

  /** Call periodically to update drivetrain with camera info */
  public void updatePosition(SwerveDrivetrain drivetrain)
  {
    --successes;
    if (successes < 0)
      successes = 0;
    if (! camera.isConnected())
    {
      // System.out.println("Camera " + camera.getName() + " is disconnected!!!");
      nt_flag.setBoolean(successes > 0);
      return;
    }

    // TODO Try estimator once more?
    // // PoseStrategy.CLOSEST_TO_REFERENCE_POSE needs to know where we think we are...
    // estimator.setReferencePose(drivetrain.getPose());
    // for (PhotonPipelineResult result : camera.getAllUnreadResults())
    //   estimator.update(result)
    //            .ifPresent(estimate ->
    //   {
    //     drivetrain.updateLocationFromCamera(estimate.estimatedPose.toPose2d(),
    //                                         result.getTimestampSeconds());
    //     successes = 50; // 1 second
    //   });

    double timestamp = 0.0;
    List<PhotonTrackedTarget> targets = new ArrayList<>();
    for (PhotonPipelineResult result : camera.getAllUnreadResults())
      if (result.hasTargets())
      {
        for (PhotonTrackedTarget target : result.getTargets())
          targets.add(target);
        timestamp = result.getTimestampSeconds();
      }
    for (PhotonTrackedTarget target : targets)
    {
      // How far is the target?
      if (target == null  ||
          target.bestCameraToTarget.getTranslation().getNorm() > 3.0)
      {
        // System.out.println("No best target");
        continue;
      }

      // Where is that tag on the field?
      Optional<Pose3d> tag_pose = tags.getTagPose(target.fiducialId);
      if (tag_pose.isEmpty())
        continue;

      // System.out.println(target.bestCameraToTarget);
      // Transform from tag to camera, then from camera to center of robot
      Pose3d pose = tag_pose.get();
      pose = pose.transformBy(target.bestCameraToTarget.inverse());
      pose = pose.transformBy(robotToCam.inverse());
      Pose2d position = pose.toPose2d();
      // System.out.println(target.getFiducialId() + " @ " + tag_pose + " -> " + position);

      // For tests, force odometry to camera reading
      // drivetrain.setOdometry(position.getX(), position.getY(), position.getRotation().getDegrees());

      // For operation, smoothly update location with camera info
      drivetrain.updateLocationFromCamera(position, timestamp);
      successes = 50; // 1 second
    }
    nt_flag.setBoolean(successes > 0);
  }
}
