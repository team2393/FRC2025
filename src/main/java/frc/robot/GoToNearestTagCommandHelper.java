// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
 *  Compares robot location to all 'reef' tags,
 *  finds the closest one,
 *  creates trajectory to get there.
 * 
 *  Does not directly use the camera,
 *  but relies on odometry knowing the current robot location,
 *  ideally updated by camera readings.
 */
public class GoToNearestTagCommandHelper
{
  private final AprilTagFieldLayout tags;
  private final Translation2d blue_reef_center, red_reef_center;
  
  /** @param tags Information about all the april tags on the field */
  public GoToNearestTagCommandHelper(AprilTagFieldLayout tags)
  {
    this.tags = tags;

    // Get reef centers.
    // Tags 18 and 21 are around the blue reef,
    // tags 7 and 10 are around the red reef.
    Pose3d p1 = tags.getTagPose(18).get();
    Pose3d p2 = tags.getTagPose(21).get();
    blue_reef_center = new Translation2d((p1.getX() + p2.getX())/2,
                                         (p1.getY() + p2.getY())/2);
    // System.out.println("Blue reef: " + blue_reef_center);

    p1 = tags.getTagPose(7).get();
    p2 = tags.getTagPose(10).get();
    red_reef_center = new Translation2d((p1.getX() + p2.getX())/2,
                                        (p1.getY() + p2.getY())/2);
    // System.out.println("Red reef: " + red_reef_center);
  }

  /** @param robot_pose Current robot position
   *  @return Nearest 'reef' april tag
   */
  private AprilTag findNearestReefTag(Pose2d robot_pose)
  {
    double nearest = Double.MAX_VALUE;
    AprilTag nearest_tag = null;
    for (AprilTag tag : tags.getTags())
    {
      // Is this a reef tag, i.e.: Within ~1m from center of a reef?
      Translation2d tag_pos = tag.pose.toPose2d().getTranslation();
      double reef_distance = Math.min(tag_pos.getDistance(blue_reef_center),
                                      tag_pos.getDistance(red_reef_center));
      // System.out.printf("#%2d: X=%5.2f, Y=%5.2f, %5.2f from reef\n",
      //                   tag.ID,
      //                   tag_pos.getX(),
      //                   tag_pos.getY(),
      //                   reef_distance);
      if (reef_distance > 0.9)
        continue;

      // It's a reef tag! Find the nearest one
      double tag_distance =  tag_pos.getDistance(robot_pose.getTranslation());
      if (tag_distance < nearest)
      {
        nearest = tag_distance;
        nearest_tag = tag;
      }
    }

    System.out.println("Nearest reef tag: " + nearest_tag);
    return nearest_tag;
  }

  /** @param target_tag An april tag to which we want to drive
   *  @return Our desired location relative to that tag
   */
  private Pose2d computeDestination(AprilTag target_tag)
  {
    // Destination is fundamentally the tag
    Pose2d dest = target_tag.pose.toPose2d();
    // .. but rotate 180 to face the tag, not point away from the tag
    dest = dest.rotateAround(dest.getTranslation(), Rotation2d.fromDegrees(180));
    // .. and move back a little to stand in front of the tag
    dest = dest.transformBy(new Transform2d(-0.7, 0, Rotation2d.fromDegrees(0)));
    // TODO Move a little based on a network table entry or buttonboard switch
    //      that selects the left or right column of reef spots

    System.out.println("Destination: " + dest);
    return dest;
  }

  /** Dynamically create the commands to drive to the nearest reef tag
   *  @param drivetrain .. to use for driving
   *  @return Command(s) to drive there
   */
  private Command findTagAndComputeCommands(SwerveDrivetrain drivetrain)
  {
    Pose2d robot_pose = drivetrain.getPose();
    AprilTag tag = findNearestReefTag(robot_pose);
    Pose2d destination = computeDestination(tag);

    // What's the difference in X and Y from robot to destination?
    double dx = destination.getX() - robot_pose.getX();
    double dy = destination.getY() - robot_pose.getY();
    
    // Distance, angle relative to the current robot heading
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
      {
        // For close-in moves this tends to fail...
        ex.printStackTrace();
      }
    }
    // May have trajectory. Follow up with rotation & swerve to exact destination
    sequence.addCommands(new RotateToHeadingCommand(drivetrain, destination.getRotation().getDegrees()));
    sequence.addCommands(new SwerveToPositionCommand(drivetrain, destination.getX(), destination.getY()));
    return sequence;
  }

  /** @param drivetrain .. to use for driving
   *  @return Deferred command that when invoked will dynamically compute the actual commands
   */
  public Command createCommand(SwerveDrivetrain drivetrain)
  {
    return new DeferredCommand(() -> findTagAndComputeCommands(drivetrain),
                               Set.of(drivetrain));
  }

  /** Test code, can run without robot */
  public static void main(String[] args)
  {
    // Near tag 17
    Pose2d robot_pose = new Pose2d(4.07-0.5, 3.31-0.5, Rotation2d.fromDegrees(10));

    GoToNearestTagCommandHelper go = new GoToNearestTagCommandHelper(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));
    AprilTag tag = go.findNearestReefTag(robot_pose);
    go.computeDestination(tag);
  }
}
