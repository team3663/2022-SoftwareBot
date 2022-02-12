package frc.robot.paths;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

public class ExampleTrajectory {

  private static ExampleTrajectory instance;

  public static ExampleTrajectory getInstance(){
    if (instance == null) {
      instance = new ExampleTrajectory();
  }
  return instance;
}

    public void generateExampleTrajectory() {
  
    var startPoint = new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0),
        Rotation2d.fromDegrees(0));
    var endPoint = new Pose2d(Units.feetToMeters(1.0), Units.feetToMeters(0),
        Rotation2d.fromDegrees(0));

    //empty arraylist of interior waypoints. Expected outcome should be a straight line.
    var interiorWaypoints = new ArrayList<Translation2d>();
      interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
      interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

  
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(0));
    config.setReversed(false);

    
  
      var trajectory = TrajectoryGenerator.generateTrajectory(
          startPoint,
          interiorWaypoints,
          endPoint,
          config);
    }


  }
