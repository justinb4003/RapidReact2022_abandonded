// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  // TODO: Robot constants will need to be tuned
  public static final double kMaxSpeed = 15.6 * 12;  // inches per second  4.758; // meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // TODO: Location of wheels from center of robot need to be defined
  private final static double halfWheelBase = 23.25/2.0;
  private final static double halfTrackWidth = 21.875/2.0;
  private final Translation2d m_frontLeftLocation = new Translation2d(halfWheelBase, halfTrackWidth);
  private final Translation2d m_frontRightLocation = new Translation2d(halfWheelBase, -halfTrackWidth);
  private final Translation2d m_backLeftLocation = new Translation2d(-halfWheelBase, halfTrackWidth);
  private final Translation2d m_backRightLocation = new Translation2d(-halfWheelBase, -halfWheelBase);

  // TOD: CAN Bus IDs need to be defined to match physical robot
  // ... and be placed in a nice central location like 'RobotMap'
  private final SwerveModule m_frontLeft = new SwerveModule(10, 20, 0, "Front Left", true);
  private final SwerveModule m_backLeft = new SwerveModule(11, 21, 1, "Back Left", true);
  private final SwerveModule m_backRight = new SwerveModule(12, 22, 2, "Back Right", true);
  private final SwerveModule m_frontRight = new SwerveModule(13, 23, 3, "Front Right", true);

  // TODO: Will need to replace the gyro here with the navX system
  private final AHRS gyro = new AHRS(Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, gyro.getRotation2d());

  public Drivetrain() {
    gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  public void resetTurnEncoders() {
    m_frontLeft.resetTurnEncoder();
    m_backLeft.resetTurnEncoder();
    m_backRight.resetTurnEncoder();
    m_frontRight.resetTurnEncoder();
  }

  public void periodic() {
    //resetTurnEncoders();
    /*
    System.out.println(m_frontLeft.getTurnPosition() + " " + m_frontRight.getTurnPosition() + " " + 
    m_backLeft.getTurnPosition() + " " + m_backRight.getTurnPosition());
    */
    System.out.println(gyro.getRotation2d());
    SmartDashboard.putNumber("Back Right", m_backRight.getTurnPosition());
    SmartDashboard.putNumber("Back Left", m_backLeft.getTurnPosition());
    SmartDashboard.putNumber("Front Right", m_frontRight.getTurnPosition());
    SmartDashboard.putNumber("Front Left", m_frontLeft.getTurnPosition());
  }
}
