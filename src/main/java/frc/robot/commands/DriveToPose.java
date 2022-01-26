// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain;
import frc.robot.Robot;

public class DriveToPose extends CommandBase {
  /** Creates a new DriveToPose. */
  double targetX, targetY, targetAngle;
  double distance;
  double lastDistance = Double.POSITIVE_INFINITY;
  boolean finished = false;
  double maxPower;
  double driveFactor = 0;
  double rotFactor = 0;
  double rampDistance = 30;
  public DriveToPose(double x, double y, double angle, double maxPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_swerve);
    targetX = x;
    targetY = y;
    targetAngle = angle;
    this.maxPower = maxPower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("command Init"); 
   }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // determine distance and heading

    Pose2d currentPose = Robot.m_swerve.getOdometry().getPoseMeters();
    double deltax = targetX - currentPose.getX();
    double deltay = targetY - currentPose.getY();
    distance = Math.sqrt(deltax * deltax + deltay * deltay);

    // determine rotation

    Rotation2d poseAngle = currentPose.getRotation();
    double deltaAngle = targetAngle - poseAngle.getDegrees();
    while (deltaAngle < -180) deltaAngle = deltaAngle + 360;
    while (deltaAngle > 180) deltaAngle = deltaAngle - 360;

    // set up power readings

    double finalRamp = Math.min(1, distance/rampDistance);
    double drivePower = maxPower * driveFactor * finalRamp;
    double ySpeed = deltay / distance * drivePower * Drivetrain.kMaxSpeed;
    double xSpeed = deltax / distance * drivePower * Drivetrain.kMaxSpeed;
    double rotationSpeed = deltaAngle / 180 * Drivetrain.kMaxAngularSpeed * rotFactor;

    // set drive power
    System.out.println(xSpeed + " " + ySpeed + " " + rotationSpeed);
    System.out.println(distance + " " + lastDistance);
    Robot.m_swerve.drive(xSpeed, ySpeed, rotationSpeed, true);

    // check to see if we're finished
    finished = distance < 20 && distance > lastDistance;
    lastDistance = distance;

    // update ramp up 
    driveFactor += 0.1;
    rotFactor += 0.1;
    if (driveFactor > 1) driveFactor = 1;
    if (rotFactor > 1) rotFactor = 1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end");
    Robot.m_swerve.stopDriveMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(finished);
     return finished;
  }
}
