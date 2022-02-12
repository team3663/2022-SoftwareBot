package frc.robot.paths;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
<<<<<<< HEAD
import edu.wpi.first.math.spline.PoseWithCurvature;
import edu.wpi.first.math.trajectory.Trajectory;
=======
>>>>>>> ece3efe34b1881c6816cc9caf72ed41e8e6c0858
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;

<<<<<<< HEAD
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

    
=======
class ExampleTrajectory {
    public void generateTrajectory() {
  
      // 2018 cross scale auto waypoints.
      // var sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
      //     Rotation2d.fromDegrees(-180));
      // var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
      //     Rotation2d.fromDegrees(-160));
      
      // try it ourselves
      var startPoint = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), Rotation2d.fromDegrees(0));
      var endPoint = new Pose2d(Units.feetToMeters(1.5), Units.feetToMeters(0.0), Rotation2d.fromDegrees(0));

      // example code
      // var interiorWaypoints = new ArrayList<Translation2d>();
      // interiorWaypoints.add(new Translation2d(Units.feetToMeters(1.5), Units.feetToMeters(0)));
      // interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));

      // try it ourselves
      var interiorWaypoints = new ArrayList<Translation2d>();
      interiorWaypoints.add(new Translation2d(Units.feetToMeters(1.0), Units.feetToMeters(0)));
      // interiorWaypoints.add(new Translation2d(Units.feetToMeters(2.0), Units.feetToMeters(0.0)));

      // TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
      // config.setReversed(true);

      // try it ourselves
      TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
      config.setReversed(true);
>>>>>>> ece3efe34b1881c6816cc9caf72ed41e8e6c0858
  
      // var trajectory = TrajectoryGenerator.generateTrajectory(
      //     sideStart,
      //     interiorWaypoints,
      //     crossScale,
      //     config);

      // try it ourselves
      var trajectory = TrajectoryGenerator.generateTrajectory(
          startPoint,
          interiorWaypoints,
          endPoint,
          config);
    }


  }
