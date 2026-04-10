package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final DifferentialDrive drive;

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private final MutVoltage appliedVoltage = Volts.mutable(0);
  private final MutDistance leftPosition = Meters.mutable(0);
  private final MutLinearVelocity leftVelocity = MetersPerSecond.mutable(0);
  private final MutDistance rightPosition = Meters.mutable(0);
  private final MutLinearVelocity rightVelocity = MetersPerSecond.mutable(0);

  private final DoubleLogEntry headingDegLog;
  private final DoubleLogEntry angularRateDegPerSecLog;

  private final SysIdRoutine rotationSysIdRoutine;

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

    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
    config.idleMode(IdleMode.kBrake);

    // Convert Spark encoder units to meters and meters/sec.
    config.encoder.positionConversionFactor(
        (Math.PI * WHEEL_DIAMETER_METERS) / DRIVE_GEAR_RATIO);
    config.encoder.velocityConversionFactor(
        ((Math.PI * WHEEL_DIAMETER_METERS) / DRIVE_GEAR_RATIO) / 60.0);

    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.disableFollowerMode();
    config.inverted(true);
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(false);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder = leftLeader.getEncoder();
    rightEncoder = rightLeader.getEncoder();

    gyro.reset();

    headingDegLog = new DoubleLogEntry(DataLogManager.getLog(), "/sysid/drive/heading_deg");
    angularRateDegPerSecLog =
        new DoubleLogEntry(DataLogManager.getLog(), "/sysid/drive/angular_rate_deg_per_sec");

    rotationSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                volts -> setTankVoltages(volts.in(Volts), -volts.in(Volts)),
                log -> {
                  log.motor("drive-left")
                      .voltage(
                          appliedVoltage.mut_replace(
                              leftLeader.get() * RobotController.getBatteryVoltage(), Volts))
                      .linearPosition(leftPosition.mut_replace(getLeftMeters(), Meters))
                      .linearVelocity(leftVelocity.mut_replace(getLeftMetersPerSecond(), MetersPerSecond));

                  log.motor("drive-right")
                      .voltage(
                          appliedVoltage.mut_replace(
                              rightLeader.get() * RobotController.getBatteryVoltage(), Volts))
                      .linearPosition(rightPosition.mut_replace(getRightMeters(), Meters))
                      .linearVelocity(rightVelocity.mut_replace(getRightMetersPerSecond(), MetersPerSecond));
                },
                this));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SysId/LeftMeters", getLeftMeters());
    SmartDashboard.putNumber("SysId/RightMeters", getRightMeters());
    SmartDashboard.putNumber("SysId/LeftMetersPerSec", getLeftMetersPerSecond());
    SmartDashboard.putNumber("SysId/RightMetersPerSec", getRightMetersPerSecond());
    SmartDashboard.putNumber("SysId/HeadingDeg", gyro.getAngle());
    SmartDashboard.putNumber("SysId/AngularRateDegPerSec", gyro.getRate());

    headingDegLog.append(gyro.getAngle());
    angularRateDegPerSecLog.append(gyro.getRate());
  }

  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }

  public void stop() {
    setTankVoltages(0.0, 0.0);
  }

  public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction) {
    return rotationSysIdRoutine.quasistatic(direction).withName("drive-rotation-quasistatic");
  }

  public Command sysIdRotationDynamic(SysIdRoutine.Direction direction) {
    return rotationSysIdRoutine.dynamic(direction).withName("drive-rotation-dynamic");
  }

  private void setTankVoltages(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  private double getLeftMeters() {
    return leftEncoder.getPosition();
  }

  private double getRightMeters() {
    return rightEncoder.getPosition();
  }

  private double getLeftMetersPerSecond() {
    return leftEncoder.getVelocity();
  }

  private double getRightMetersPerSecond() {
    return rightEncoder.getVelocity();
  }
}