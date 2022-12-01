// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  // private final Spark m_leftMotor = new Spark(0);
  // private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  // private final Encoder m_leftEncoder = new Encoder(4, 5);
  // private final Encoder m_rightEncoder = new Encoder(6, 7);

  private final WPI_TalonFX m_leftLeader = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainLeftBackTalonFX);
  private final WPI_TalonFX m_rightLeader = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainRightBackTalonFX);
  private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainLeftFrontTalonFX);
  private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainRightFrontTalonFX);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  /** Creates a new Drivetrain. */
  public Drivetrain(Supplier<Transmission.GearState> gearStateSupplier) {

     // Motors
     configmotors();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    // m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    // m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    // resetEncoders();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  // public void resetEncoders() {
  //   m_leftEncoder.reset();
  //   m_rightEncoder.reset();
  // }

  // public int getLeftEncoderCount() {
  //   return m_leftEncoder.get();
  // }

  // public int getRightEncoderCount() {
  //   return m_rightEncoder.get();
  // }

  // public double getLeftDistanceInch() {
  //   return m_leftEncoder.getDistance();
  // }

  // public double getRightDistanceInch() {
  //   return m_rightEncoder.getDistance();
  // }

  // public double getAverageDistanceInch() {
  //   return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  // }

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

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  public void configmotors() {


    // Configure the motors
    for(TalonFX fx : new TalonFX[] {m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower}){
        //Reset settings for safety
        fx.configFactoryDefault();

        //Sets voltage compensation to 12, used for percent output
        fx.configVoltageCompSaturation(10);
        fx.enableVoltageCompensation(true);

        //Setting just in case
        fx.configNominalOutputForward(0);
        fx.configNominalOutputReverse(0);
        fx.configPeakOutputForward(1);
        fx.configPeakOutputReverse(-1);

        fx.configOpenloopRamp(0.1);

        //Setting deadband(area required to start moving the motor) to 1%
        fx.configNeutralDeadband(0.01);

        //Set to brake mode, will brake the motor when no power is sent
        fx.setNeutralMode(NeutralMode.Coast);

        /** 
         * Setting input side current limit (amps)
         * 45 continious, 80 peak, 30 millieseconds allowed at peak
         * 40 amp breaker can support above 40 amps for a little bit
         * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
         */
        fx.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

        //Either using the integrated Falcon sensor or an external one, will change if needed
        fx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
