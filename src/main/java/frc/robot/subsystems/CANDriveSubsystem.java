// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANDriveSubsystem extends SubsystemBase {
  private static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 in
  private static final double DRIVE_GEAR_RATIO = 10.71;
  private static final double TRACK_WIDTH_METERS = 0.60;

  private static final double POSITION_CONVERSION_FACTOR =
      Math.PI * WHEEL_DIAMETER_METERS / DRIVE_GEAR_RATIO;
  private static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;

  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final DifferentialDrive drive;
  private final ADIS16448_IMU gyro;
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(TRACK_WIDTH_METERS);
  private final DifferentialDriveOdometry odometry;

  private final PIDController leftVelocityPid = new PIDController(1.6, 0.0, 0.0);
  private final PIDController rightVelocityPid = new PIDController(1.6, 0.0, 0.0);
  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(0.20, 2.20);

  public CANDriveSubsystem() {
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    drive = new DifferentialDrive(leftLeader, rightLeader);

    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    leftLeaderConfig.voltageCompensation(12);
    leftLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    leftLeaderConfig.idleMode(IdleMode.kBrake);
    leftLeaderConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR);
    leftLeaderConfig.encoder.velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    rightLeaderConfig.voltageCompensation(12);
    rightLeaderConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    rightLeaderConfig.idleMode(IdleMode.kBrake);
    rightLeaderConfig.inverted(true);
    rightLeaderConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR);
    rightLeaderConfig.encoder.velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    leftFollowerConfig.follow(leftLeader);
    leftFollowerConfig.voltageCompensation(12);
    leftFollowerConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    leftFollowerConfig.idleMode(IdleMode.kBrake);

    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.follow(rightLeader);
    rightFollowerConfig.voltageCompensation(12);
    rightFollowerConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    rightFollowerConfig.idleMode(IdleMode.kBrake);

    leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    gyro = new ADIS16448_IMU();
    gyro.calibrate();

    odometry = new DifferentialDriveOdometry(
        getHeading(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition()
    );

    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load PathPlanner robot config", e);
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPLTVController(0.02),
        robotConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        },
        this
    );
  }

  private Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);
    odometry.resetPosition(
        getHeading(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition(),
        pose
    );
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity(),
            rightEncoder.getVelocity()
        )
    );
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    tankDriveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards ignoredFeedforwards) {
    driveRobotRelative(speeds);
  }

  public void tankDriveVelocity(double leftMetersPerSecond, double rightMetersPerSecond) {
    double leftVolts =
        feedforward.calculate(leftMetersPerSecond)
            + leftVelocityPid.calculate(leftEncoder.getVelocity(), leftMetersPerSecond);
    double rightVolts =
        feedforward.calculate(rightMetersPerSecond)
            + rightVelocityPid.calculate(rightEncoder.getVelocity(), rightMetersPerSecond);

    tankDriveVolts(leftVolts, rightVolts);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    drive.feed();
  }

  public void stop() {
    tankDriveVolts(0.0, 0.0);
  }

  @Override
  public void periodic() {
    odometry.update(
        getHeading(),
        leftEncoder.getPosition(),
        rightEncoder.getPosition()
    );
  }
}