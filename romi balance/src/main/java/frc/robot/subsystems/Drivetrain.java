// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final PIDController leftDrivePIDController = new PIDController(Constants.kP_speed, Constants.kI_speed, Constants.kD_speed);
  private final PIDController rightDrivePIDController = new PIDController(Constants.kP_speed, Constants.kI_speed, Constants.kD_speed);
  private double leftDesiredOutput;
  private double rightDesiredOutput;

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  NetworkTableInstance m_insta = NetworkTableInstance.getDefault();
  NetworkTable m_pidTable = m_insta.getTable("PID_Table");
  NetworkTable m_drivetrainTable = m_insta.getTable("Drivetrain_Table");

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    m_pidTable.getEntry("kP").setDouble(Constants.kP_speed);
    m_pidTable.getEntry("kI").setDouble(Constants.kI_speed);
    m_pidTable.getEntry("kD").setDouble(Constants.kD_speed);
    

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_drivetrainTable.getEntry("Left_Current_Speed").setDouble(getLeftEncoderRateMps());
    m_drivetrainTable.getEntry("Right_Current_Speed").setDouble(getRightEncoderRateMps());

    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
    SmartDashboard.putNumber("Power level we want", xaxisSpeed);
  }

  public void pidDrive(double leftSpeed, double rightSpeed) {
    m_pidTable.getEntry("Left_Current_Speed").setDouble(getLeftEncoderRateMps());
    m_pidTable.getEntry("Right_Current_Speed").setDouble(getRightEncoderRateMps());

    m_drivetrainTable.getEntry("Gyro_Rate_X").setDouble(getGyroRateX());
    m_drivetrainTable.getEntry("Gyro_Rate_Y").setDouble(getGyroRateY());
    m_drivetrainTable.getEntry("Gyro_Rate_Z").setDouble(getGyroRateZ());

    m_drivetrainTable.getEntry("Gyro_Angle_X").setDouble(getGyroAngleX());
    m_drivetrainTable.getEntry("Gyro_Angle_Y").setDouble(getGyroAngleY());
    m_drivetrainTable.getEntry("Gyro_Angle_Z").setDouble(getGyroAngleZ());
    

    leftDrivePIDController.setP(m_pidTable.getEntry("kP").getDouble(0.05));
    leftDrivePIDController.setI(m_pidTable.getEntry("kI").getDouble(0));
    leftDrivePIDController.setD(m_pidTable.getEntry("kD").getDouble(0));

    rightDrivePIDController.setP(m_pidTable.getEntry("kP").getDouble(0.05));
    rightDrivePIDController.setI(m_pidTable.getEntry("kI").getDouble(0));
    rightDrivePIDController.setD(m_pidTable.getEntry("kD").getDouble(0));
    

    leftDesiredOutput = leftDrivePIDController.calculate(getLeftEncoderRateMps(), leftSpeed);
    rightDesiredOutput = rightDrivePIDController.calculate(getRightEncoderRateMps(), rightSpeed);

    m_pidTable.getEntry("Left_Desired_Speed").setDouble(leftSpeed);
    m_pidTable.getEntry("Right_Desired_Speed").setDouble(rightSpeed);

    m_diffDrive.tankDrive(leftDesiredOutput, rightDesiredOutput);
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

  public double getLeftEncoderRateMps() {
    return m_leftEncoder.getRate()/39.37;
  } 

  public double getRightEncoderRateMps() {
    return m_rightEncoder.getRate()/39.37;
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
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  public double getGyroRateX() {
    return m_gyro.getRateX();
  }

  public double getGyroRateY() {
    return m_gyro.getRateY();
  }

  public double getGyroRateZ() {
    return m_gyro.getRateZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
