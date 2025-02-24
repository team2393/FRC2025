// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import frc.swervelib.SwerveDrivetrain;

/** Helper for using a camera */
public class CameraHelper
{
  private final AprilTagFieldLayout tags;
  private final PhotonCamera camera;
  private final Transform3d robotToCam;
  private final PhotonPoseEstimator estimator;
    
  public CameraHelper ()
  {
    // TODO pick correct field
    tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    // TODO Configure camera name and position
    camera = new PhotonCamera("photonvision");

    // TODO: Allow access to the camera from a computer when tethered to the USB port on the roboRIO
    // PortForwarder.add(5800, "photonvision.local", 5800);

    // Where is the camera mounted relative to the center of the robot?
    // Example: mounted facing forward, half a meter forward of center, half a meter up from center.
    robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

    // Prepare estimator
    // TODO Which strategy?
    estimator = new PhotonPoseEstimator(tags, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
  }

  /** Call periodically to update drivetrain with camera info */
  public void updatePosition(SwerveDrivetrain drivetrain)
  {
    if (! camera.isConnected())
      return;
    
    // PoseStrategy.CLOSEST_TO_REFERENCE_POSE needs to know where we think we are...
    estimator.setReferencePose(drivetrain.getPose());
    for (PhotonPipelineResult info : camera.getAllUnreadResults())
    {
      // Option 1: Trust the estimator
      Optional<EstimatedRobotPose> estimate = estimator.update(info);
      if (estimate.isPresent())
        drivetrain.updateLocationFromCamera(estimate.get().estimatedPose.toPose2d(),
                                            estimate.get().timestampSeconds);

      // Option 2: Check the result outself...
      // PhotonTrackedTarget target = info.getBestTarget();
      // // How far is the target?
      // if (target.bestCameraToTarget.getTranslation().getNorm() > 1.0)
      //   continue;
      // // Where is that tag on the field?
      // Optional<Pose3d> tag_pose = tags.getTagPose(target.fiducialId);
      // if (tag_pose.isEmpty())
      //   continue;
      // // Transform from tag to camera, then from camera to center of robot
      // Pose2d position = tag_pose.get()
      //                           .transformBy(target.bestCameraToTarget.inverse())
      //                           .transformBy(robotToCam.inverse())
      //                           .toPose2d();
      // drivetrain.updateLocationFromCamera(position, info.getTimestampSeconds());
    }
  }  
}
