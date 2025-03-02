package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.swervelib.SwerveDrivetrain;

public class GoToNearestTagCommand extends Command
{
  private final AprilTagFieldLayout tags;
  private final SwerveDrivetrain drivetrain;
  private final Translation2d blue_reef, red_reef;
  
  public GoToNearestTagCommand(AprilTagFieldLayout tags, SwerveDrivetrain drivetrain)
  {
    this.tags = tags;
    this.drivetrain = drivetrain;

    // Get reef centers.
    // Tags 18 and 21 are around the blue reef,
    // tags 7 and 10 are around the red reef.
    Pose3d p1 = tags.getTagPose(18).get();
    Pose3d p2 = tags.getTagPose(21).get();
    blue_reef = new Translation2d((p1.getX() + p2.getX())/2,
                                  (p1.getY() + p2.getY())/2);
    System.out.println("Blue reef: " + blue_reef);

    p1 = tags.getTagPose(7).get();
    p2 = tags.getTagPose(10).get();
    red_reef = new Translation2d((p1.getX() + p2.getX())/2,
                                 (p1.getY() + p2.getY())/2);
    System.out.println("Red reef: " + red_reef);
  }

  public AprilTag findNearestTag(Pose2d robot_pose)
  {
    double nearest = Double.MAX_VALUE;
    AprilTag nearest_tag = null;
    for (AprilTag tag : tags.getTags())
    {
      // Is this a reef tag?
      // Ignore all that are more then ~1m from a reef
      Translation2d tag_pos = tag.pose.toPose2d().getTranslation();
      double reef_distance = Math.min(red_reef.getDistance(tag_pos),
                                      blue_reef.getDistance(tag_pos));
      System.out.printf("#%2d: X=%5.2f, Y=%5.2f, %5.2f from reef\n",
                        tag.ID,
                        tag_pos.getX(),
                        tag_pos.getY(),
                        reef_distance);
      if (reef_distance >= 0.9)
        continue;

      double tag_distance = robot_pose.getTranslation().getDistance(tag_pos);
      if (tag_distance < nearest)
      {
        nearest = tag_distance;
        nearest_tag = tag;
      }
    }

    System.out.println("Nearest tag: " + nearest_tag);
    return nearest_tag;
  }

  public Pose2d computeDestination(AprilTag target_tag)
  {
    // Destination is fundamentally the tag
    Pose2d dest = target_tag.pose.toPose2d();
    // .. but rotate 180 to face the tag, not point away from the tag
    dest = dest.rotateAround(dest.getTranslation(), Rotation2d.fromDegrees(180));

    // .. and move back a little to stand in front of the tag
    dest = dest.transformBy(new Transform2d(-1.1, 0, Rotation2d.fromDegrees(0)));

    System.out.println("Destination: " + dest);
    return dest;
  }

  @Override
  public void execute()
  {
    Pose2d robot_pose = drivetrain.getPose();
    AprilTag tag = findNearestTag(robot_pose);
    Pose2d destination = computeDestination(tag);
    drivetrain.setOdometry(destination.getX(), destination.getY(), destination.getRotation().getDegrees());
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }

  public static void main(String[] args)
  {
    // Near tag 17
    Pose2d robot_pose = new Pose2d(4.07-0.5, 3.31-0.5, Rotation2d.fromDegrees(10));

    GoToNearestTagCommand go = new GoToNearestTagCommand(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded), null);
    AprilTag tag = go.findNearestTag(robot_pose);
    Pose2d destination = go.computeDestination(tag);
  }
}
