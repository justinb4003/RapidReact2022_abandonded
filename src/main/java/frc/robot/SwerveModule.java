// Copyright (c) FIRST and other WPILib contrinutors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveModule {
  // TODO: These constants need to be defined per robot.
  private static final double kWheelRadius = 2; // inches
  private static final double kDriveGearRatio = 7.131;
  private static final double kEffectiveRadius = kWheelRadius / kDriveGearRatio;
  private static final int kDriveResolution =  2048;
  private static final int kTurnResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;
  private AnalogInput m_turnEncoder;
  private int m_encoderOffset = 0;

  //private final Encoder m_driveEncoder;
  //private final Encoder m_turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  // TODO: Need to tune the PID controller here, a P of 1 is usually quite high.
  private final PIDController m_turnPIDController = new PIDController(1/1000.0, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  // TODO: Need to tune the PID controller here, a P of 1 is usually quite high.
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  // TODO: Also will need tuning
  //private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  //private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel CAN ID for the drive motor.
   * @param turningMotorChannel CAN ID for the turning motor.
   * 
   */

   boolean driveDisabled = false;
   String name;
   public SwerveModule(int driveMotorID, int turningMotorID, int turnEncoderChannel, int turnEncoderoffset, String name, boolean driveDisabled) {
     this(driveMotorID, turningMotorID, turnEncoderChannel, turnEncoderoffset,  name);
     this.driveDisabled = driveDisabled;
   }
  public SwerveModule(int driveMotorID, int turningMotorID, int turnEncoderChannel, int turnEncoderoffset, String name) {
    // TODO: The controllers won't be SparkMax, this needs to change.
    m_driveMotor = new TalonFX(driveMotorID);
    m_turningMotor = new TalonFX(turningMotorID);

    m_driveMotor.setNeutralMode(NeutralMode.Coast);
    m_encoderOffset = turnEncoderoffset;
    m_driveMotor.config_kF(0, 0.0465);
    m_driveMotor.config_kP(0, 0.0);
    m_turningMotor.config_kP(0, 0.1);
    m_turningMotor.config_kD(0, 0.002);
    m_turningMotor.setInverted(TalonFXInvertType.Clockwise);

    m_turnEncoder = new AnalogInput(turnEncoderChannel);
    
    this.name = name;
    // TODO: Much like the motor controllers the Encoders are going to have to change
    //m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    //m_turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kEffectiveRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }
    public double getDriveVelocity(){
      return m_driveMotor.getSelectedSensorVelocity() * 10/kDriveResolution * 2 * Math.PI * kEffectiveRadius;
    }
    public double getTurnPositionInradians(){
      return getTurnPosition()/kTurnResolution * 2 * Math.PI;
    }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(),
    new Rotation2d(getTurnPositionInradians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    double currentTurnPosition = getTurnPosition();
    double turnAngle = 2*Math.PI/kTurnResolution * currentTurnPosition;
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(turnAngle));

    

    // Calculate the drive output from the drive PID controller.
    //final double driveOutput =
    //    m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);

    double driveVelocity = state.speedMetersPerSecond / (2*Math.PI*kEffectiveRadius) * kDriveResolution * 0.1;
    
    //final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    //if (name == "Front Right") 
      //  System.out.println(name + " " + state.angle.getRadians() + " " + turnAngle);
    double setPosition = state.angle.getRadians()/(2*Math.PI) * kTurnResolution;
    double revs = Math.round(currentTurnPosition / kTurnResolution);
    setPosition += revs * kTurnResolution;
    while (setPosition > currentTurnPosition + kTurnResolution / 2) setPosition -= kTurnResolution;
    while (setPosition < currentTurnPosition - kTurnResolution / 2) setPosition += kTurnResolution;

    // Calculate the turning motor output from the turning PID controller.
    // final double turnOutput =
    //     m_turningPIDController.calculate(m_turningMotor.getSelectedSensorPosition(), state.angle.getRadians()); 

    //final double turnFeedforward =
    //    m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    //if (!driveDisabled) 
    m_driveMotor.set(TalonFXControlMode.Velocity, driveVelocity);
    m_turnPIDController.setSetpoint(setPosition);
    double turnPower = m_turnPIDController.calculate(currentTurnPosition);
    m_turningMotor.set(TalonFXControlMode.PercentOutput, turnPower);
    //System.out.println(setPosition + " " + currentTurnPosition);
    //m_turningMotor.set(TalonFXControlMode.Position, turnOutput);
  }

  public void resetTurnEncoder() {
    m_encoderOffset = m_turnEncoder.getValue();
  }

  public double getTurnPosition() {
    return m_turnEncoder.getValue()-m_encoderOffset;
  }

  public void stopDriveMotor() {
    m_driveMotor.set(TalonFXControlMode.PercentOutput, 0);
    m_turningMotor.set(TalonFXControlMode.PercentOutput, 0);
  }
}
