// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.RotateToHeadingCommand;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveToPositionCommand;
import frc.tools.AutoTools;

/** Helper for creating command that drives to the nearest useful tag
 * 
 *  Compares robot location to all 'tags of interest',
 *  finds the closest one,
 *  tweaks destination a little for reef vs. pickup station,
 *  creates trajectory to get there.
 * 
 *  Does not directly use the camera,
 *  but relies on odometry knowing the current robot location,
 *  ideally updated by camera readings.
 */
public class GoToNearestTagCommandHelper
{
  // Reef tag IDs
  private static final Set<Integer> reef_tags = Set.of( 6,  7,  8,  9, 10, 11,  // red reef
                                                       17, 18, 19, 20, 21, 22); // blue reef
  // Pickup station tag IDs
  private static final Set<Integer> pickup_tags = Set.of( 1,  2,  // red pickup
                                                         12, 13); // blue pickup
  // All the tags of interest
  private static final Set<Integer> tags_of_interest = new HashSet<>();
  static
  {
    tags_of_interest.addAll(pickup_tags);
    tags_of_interest.addAll(reef_tags);
  }

  // Info about all the tags on the field
  private final AprilTagFieldLayout tags;
  
  /** @param tags Information about all the april tags on the field */
  public GoToNearestTagCommandHelper(AprilTagFieldLayout tags)
  {
    this.tags = tags;
  }

  /** @param robot_pose Current robot position
   *  @return Nearest tag of interest
   */
  private AprilTag findNearestTag(Pose2d robot_pose)
  {
    double nearest = Double.MAX_VALUE;
    AprilTag nearest_tag = null;
    for (AprilTag tag : tags.getTags())
      if (tags_of_interest.contains(tag.ID))
      { // It's a tag of interest! Find the nearest one
        Translation2d tag_pos = tag.pose.toPose2d().getTranslation();
        double tag_distance =  tag_pos.getDistance(robot_pose.getTranslation());
        if (tag_distance < nearest)
        {
          nearest = tag_distance;
          nearest_tag = tag;
        }
      }
    System.out.println("Nearest tag: " + nearest_tag);
    return nearest_tag;
  }

  /** @param target_tag An april tag to which we want to drive
   *  @param right_column Align with right column? Otherwise left 
   *  @return Our desired location relative to that tag
   */
  private Pose2d computeDestination(AprilTag target_tag, boolean right_column)
  {
    // Destination is fundamentally the tag
    Pose2d dest = target_tag.pose.toPose2d();
    // Adjust for reef vs. pickup station
    if (reef_tags.contains(target_tag.ID))
    { // Rotate 180 to face the tag, not point away from the tag
      dest = dest.rotateAround(dest.getTranslation(), Rotation2d.fromDegrees(180));
      
      // .. and move back in X a little to stand in front of the tag.
      // Move a little in Y to select the left or right column of reef branches
      dest = dest.transformBy(new Transform2d(-0.7,
                                              // "pipes ..are .. ~33 cm.. apart (center to center)"
                                              right_column ? -0.33/2 : +0.33/2,
                                              Rotation2d.fromDegrees(0)));
    }
    else
    { // Keep back of robot to loading station, 
      // move in X a little to stand in front & center of the tag.
      dest = dest.transformBy(new Transform2d(0.5,
                                              0,
                                              Rotation2d.fromDegrees(0)));
    }
    System.out.println("Destination: " + dest);
    return dest;
  }

  /** Dynamically create the commands to drive to the nearest reef tag
   *  @param drivetrain .. to use for driving
   *  @param right_column Align with right column? Otherwise left 
   *  @return Command(s) to drive there
   */
  private Command findTagAndComputeCommands(SwerveDrivetrain drivetrain, boolean right_column)
  {
    Pose2d robot_pose = drivetrain.getPose();
    AprilTag tag = findNearestTag(robot_pose);
    Pose2d destination = computeDestination(tag, right_column);

    // What's the difference in X and Y from robot to destination?
    double dx = destination.getX() - robot_pose.getX();
    double dy = destination.getY() - robot_pose.getY();
    
    // Distance and angle relative to the current robot heading
    double distance = Math.hypot(dx, dy);
    double heading = Math.toDegrees(Math.atan2(dy, dx)) - robot_pose.getRotation().getDegrees();
    // System.out.println("Threshold: " + distance + " @ " + heading);

    SequentialCommandGroup sequence = new SequentialCommandGroup();
    // Trajectory fails for short distances but is more efficient
    // for long path, so start with that when far away
    if (distance > 2.0)
    {
      try
      { // Try to create a trajectory
        Trajectory traj = AutoTools.createTrajectory(true,
                                                    robot_pose.getX(),  robot_pose.getY(),  heading,
                                                    destination.getX(), destination.getY(), heading);
        sequence.addCommands(drivetrain.followTrajectory(traj, destination.getRotation().getDegrees()));
      }
      catch (Exception ex)
      { // For close-in moves this tends to fail...
        ex.printStackTrace();
      }
    }
    // May have trajectory. Follow up with rotation & swerve to exact destination
    sequence.addCommands(new RotateToHeadingCommand(drivetrain, destination.getRotation().getDegrees()));
    sequence.addCommands(new SwerveToPositionCommand(drivetrain, destination.getX(), destination.getY()));
    return sequence;
  }

  /** @param drivetrain .. to use for driving
   *  @param right_column Align with right column? Otherwise left 
   *  @return Deferred command that when invoked will dynamically compute the actual commands
   */
  public Command createCommand(SwerveDrivetrain drivetrain, boolean right_column)
  {
    return new DeferredCommand(() -> findTagAndComputeCommands(drivetrain, right_column),
                               Set.of(drivetrain));
  }

  /** Test code, can run without robot */
  public static void main(String[] args)
  {
    // Near tag 17
    Pose2d robot_pose = new Pose2d(4.07-0.5, 3.31-0.5, Rotation2d.fromDegrees(10));

    GoToNearestTagCommandHelper go = new GoToNearestTagCommandHelper(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
    AprilTag tag = go.findNearestTag(robot_pose);
    go.computeDestination(tag, true);
  }
}
