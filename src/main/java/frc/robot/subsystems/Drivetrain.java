// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {

  public static CANSparkMax leftMotor1 = new CANSparkMax(DrivetrainConstants.FL, MotorType.kBrushless);
  public static CANSparkMax leftMotor2 = new CANSparkMax(DrivetrainConstants.BL, MotorType.kBrushless);

  // MotorControllerGroup m_leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);

  public static CANSparkMax rightMotor1 = new CANSparkMax(DrivetrainConstants.FR, MotorType.kBrushless);
  public static CANSparkMax rightMotor2 = new CANSparkMax(DrivetrainConstants.BR, MotorType.kBrushless);

  private final DifferentialDrive drivetrain = new DifferentialDrive(leftMotor1, rightMotor1);

  private final RelativeEncoder m_leftEncoder1 = leftMotor1.getEncoder();
  private final RelativeEncoder m_leftEncoder2 = leftMotor2.getEncoder();
  private final RelativeEncoder m_rightEncoder1 = rightMotor1.getEncoder();
  private final RelativeEncoder m_rightEncoder2 = rightMotor2.getEncoder();
  
  private final AHRS ahrs;

  private static DifferentialDriveOdometry m_odometry;



  /** Creates a new Drivetrain. */
  public Drivetrain() {
    ahrs = new AHRS(SPI.Port.kMXP);
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double xSpeed, double zRotation){
    drivetrain.arcadeDrive(xSpeed, zRotation);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public double getLeftEncoderVelocity() {
    return (m_leftEncoder1.getVelocity() + m_leftEncoder2.getVelocity()) / 2;
  }

  public double getRightEncoderVelocity() {
    return (m_rightEncoder1.getVelocity() + m_rightEncoder2.getVelocity()) / 2;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), m_leftEncoder1.getPosition(), m_rightEncoder1.getPosition(), pose);
  }

  public double getLeftEncoder1Rotations() {
    return m_leftEncoder1.getPosition();
  }

  public double getLeftEncoder2Rotations() {
    return m_leftEncoder1.getPosition();
  }

  public double getRightEncoder1Rotations() {
    return m_leftEncoder1.getPosition();
  }

  public double getRightEncoder2Rotations() {
    return m_leftEncoder1.getPosition();
  }

  public double getAverageEncoderRotations() {
    return (getLeftEncoder1Rotations()+getLeftEncoder2Rotations()+getRightEncoder1Rotations()+getRightEncoder2Rotations()) / 4;
  }

  public void zeroHeading() {
    ahrs.reset();
  }

  public void stopMotors(){
    drivetrain.stopMotor();
  }

  public double getHeading() {
    return Math.IEEEremainder(ahrs.getAngle(), 360) * (DrivetrainConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
