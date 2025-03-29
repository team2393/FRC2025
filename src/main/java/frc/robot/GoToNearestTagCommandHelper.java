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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.RotateToHeadingCommand;
import frc.swervelib.SwerveDrivetrain;
import frc.swervelib.SwerveToPositionCommand;

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

  private static NetworkTableEntry nt_load_distance, nt_load_left, nt_reef_distance, nt_reef_left;

  static
  {
    tags_of_interest.addAll(pickup_tags);
    tags_of_interest.addAll(reef_tags);

    nt_load_distance = SmartDashboard.getEntry("AutoLoadDist");
    nt_load_left = SmartDashboard.getEntry("AutoLoadLeft");
    nt_reef_distance = SmartDashboard.getEntry("AutoReefDist");
    nt_reef_left = SmartDashboard.getEntry("AutoReefLeft");
    nt_load_distance.setDefaultDouble(-0.3);
    nt_load_left.setDefaultDouble(-0.1);
    nt_reef_distance.setDefaultDouble(0);
    nt_reef_left.setDefaultDouble(-0.08);
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
    // System.out.println("Nearest tag: " + nearest_tag);
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

    // TODO Do we need to handle some specific reef face differently?
    // if (target_tag.ID == 20)
    // {
    //   dest = dest.rotateAround(dest.getTranslation(), Rotation2d.fromDegrees(180));
    //   dest = dest.transformBy(new Transform2d(-0.35,
    //                                           right_column ? -0.35/2-0.08 : +0.35/2-0.08,
    //                                           Rotation2d.fromDegrees(0)));
    // }
    // else
    // Adjust for reef vs. pickup station
    if (reef_tags.contains(target_tag.ID))
    { // Rotate 180 to face the tag, not point away from the tag
      dest = dest.rotateAround(dest.getTranslation(), Rotation2d.fromDegrees(180));

      // .. and move back in X a little to stand in front of the tag.
      // Move a little in Y to select the left or right column of reef branches
      dest = dest.transformBy(new Transform2d(-0.35 - nt_reef_distance.getDouble(0),
                                              // "pipes ..are .. ~33 cm.. apart (center to center)"
                                              right_column ? -0.35/2 + nt_reef_left.getDouble(0)
                                                           : +0.35/2 + nt_reef_left.getDouble(0),
                                              Rotation2d.fromDegrees(0)));
    }
    else
    { // Keep back of robot to loading station,
      // move in X a little to stand in front & center of the tag.
      dest = dest.transformBy(new Transform2d(0.4 + nt_load_distance.getDouble(0),
                                              0.1 + nt_load_left.getDouble(0),
                                              Rotation2d.fromDegrees(0)));
    }
    // System.out.println("Destination: " + dest);
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

    SequentialCommandGroup sequence = new SequentialCommandGroup();
    sequence.addCommands(new RotateToHeadingCommand(drivetrain, destination.getRotation().getDegrees()));
    sequence.addCommands(new SwerveToPositionCommand(drivetrain, destination.getX(),
                                                                 destination.getY(),
                                                                 destination.getRotation().getDegrees()));
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
