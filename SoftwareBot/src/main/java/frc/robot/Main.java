package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;


 // Do NOT modify this file except to change the parameter class to the startRobot call.
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
