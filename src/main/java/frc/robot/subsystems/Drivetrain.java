// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;


public class Drivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final double kP = 0.5;
  private final double kS = 0.3;
  private final double kV = 7.5;

  private final SimpleMotorFeedforward m_feedforward = 
    new SimpleMotorFeedforward(kS, kV);

  private final PIDController leftPID = new PIDController(kP, 0.0, 0.0);
  private final PIDController rightPID = new PIDController(kP, 0.0, 0.0);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Track width is ~6.1 inches for the standard XRP
private final DifferentialDriveKinematics m_kinematics = 
    new DifferentialDriveKinematics(Units.inchesToMeters(6.1));

private final DifferentialDriveOdometry m_odometry = 
    new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0);

// Simple record to hold the Pose and Speeds PathPlanner expects
public static record XRPState(Pose2d Pose, ChassisSpeeds Speeds) {}

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    double distancePerPulse = (Math.PI * 0.06)/288.0;
    m_leftEncoder.setDistancePerPulse(distancePerPulse);
    m_rightEncoder.setDistancePerPulse(distancePerPulse);
    SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    

    
    

    configureAutoBuilder();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the XRP along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the XRP along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(),
        Units.inchesToMeters(getLeftDistanceInch()),
        Units.inchesToMeters(getRightDistanceInch())
    );
  }

  private void configureAutoBuilder() {
      // State costs: [x, y, theta]
// We allow 0.05m error in X/Y and ~0.2 radians error in heading
var qelems = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(10));

// Control effort costs: [linear velocity, angular velocity]
// We allow 1.0 m/s and 2.0 rad/s of "effort"
var relems = VecBuilder.fill(1.0, 2.0);
    
        try {
            //var config = RobotConfig.fromGUISettings();

            RobotConfig config;
    try {
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        e.printStackTrace();
        return; // Don't configure AutoBuilder if this fails!
    }
            AutoBuilder.configure(
                this::getPose,               // Method to get your current Pose2d
        this::resetPose,             // Method to reset odometry
        this::getCurrentSpeeds,      // Method to get current ChassisSpeeds (Code below)
        (speeds, feedforwards) -> driveRobotRelative(speeds), // The method PathPlanner will call to drive

                new PPLTVController(qelems,relems,
                0.02,0.5
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder: " + ex.getMessage(), ex.getStackTrace());
        }
    }

  
    public XRPState getState() {
    // Convert current encoder rates from inches/sec to meters/sec
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
        Units.inchesToMeters(m_leftEncoder.getRate()), 
        Units.inchesToMeters(m_rightEncoder.getRate())
    );

    // Convert wheel speeds to robot-relative ChassisSpeeds
    ChassisSpeeds robotSpeeds = m_kinematics.toChassisSpeeds(wheelSpeeds);

    return new XRPState(m_odometry.getPoseMeters(), robotSpeeds);
}

public void resetPose(Pose2d pose) {
    resetEncoders();
    m_gyro.reset();
    m_odometry.resetPosition(m_gyro.getRotation2d(), 0, 0, pose);
}

public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
        // Get the target speeds asked for by PathPlanner (in meters per second)
        double targetLeftSpeed = speeds.leftMetersPerSecond;
        double targetRightSpeed = speeds.rightMetersPerSecond;

        // Get the current actual speeds from the XRP encoders (getRate() returns m/s)
        double actualLeftSpeed = m_leftEncoder.getRate();
        double actualRightSpeed = m_rightEncoder.getRate();

        // --- STEP A: Calculate Feedforward (The Base Voltage) ---
        double leftFF = m_feedforward.calculate(targetLeftSpeed);
        double rightFF = m_feedforward.calculate(targetRightSpeed);

        // --- STEP B: Calculate PID (The Correction Voltage) ---
        double leftPIDOutput = leftPID.calculate(actualLeftSpeed, targetLeftSpeed);
        double rightPIDOutput = rightPID.calculate(actualRightSpeed, targetRightSpeed);

        // --- STEP C: Add them together and send Volts to the motors ---
        m_leftMotor.setVoltage(leftFF + leftPIDOutput);
        m_rightMotor.setVoltage(rightFF + rightPIDOutput);

        // (Optional Debugging) - Push to SmartDashboard to tune easily
        SmartDashboard.putNumber("Auto/Left Target Speed", targetLeftSpeed);
        SmartDashboard.putNumber("Auto/Left Actual Speed", actualLeftSpeed);
    }

  /** Returns the currently-estimated pose of the robot. */
public Pose2d getPose() {
    return m_odometry.getPoseMeters();
}

/** Returns the current robot-relative speeds. */
public ChassisSpeeds getCurrentSpeeds() {
    // Convert current encoder rates (inches/s) to meters/s
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
        Units.inchesToMeters(m_leftEncoder.getRate()), 
        Units.inchesToMeters(m_rightEncoder.getRate())
    );
    // Convert wheel speeds to chassis speeds using kinematics
    return m_kinematics.toChassisSpeeds(wheelSpeeds);
}

/** Drives the robot using robot-relative ChassisSpeeds. */
public void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert ChassisSpeeds to WheelSpeeds
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    // Pass those to your existing speed control method
    setWheelSpeeds(wheelSpeeds);
}

}













