// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class PneumaticIDs {
        //Solenoids
        // public static final int kDrivetrainShiftSolenoid = 0;
        public static final int kDrivetrainShiftSolenoidLow = 2;
        public static final int kDrivetrainShiftSolenoidHigh = 3;

        public static final int kRampSolenoid = 1;
        public static final int kClimberSolenoid = 2;

        // public static final int kDrivetrainShiftSolenoidLow = 0;
        // public static final int kDrivetrainShiftSolenoidHigh = 1;
        // public static final int kRampSolenoidOpen = 2;
        // public static final int kRampSolenoidClosed = 3;
       
    }
    public static final class CANBusIDs {
        // Drivetrain, right side
        public static final int kDrivetrainRightBackTalonFX = 0;
        public static final int kDrivetrainRightFrontTalonFX = 1;

        //Shooter
        public static final int kFlywheelTalonFX = 2;

        //Sensors
        public static final int kPigeonIMU = 3;

        //Intake
        public static final int kIntakeMotor = 4;
        
        //Climber
        public static final int kClimberMotor = 5; 
        
        //Turret
        public static final int kTurretTalonSRX = 9;

        //Feeder
        public static final int kLeftFeederMotor = 11;
        public static final int kRightFeederMotor = 10;

        // Drivetrain, left side
        public static final int kDrivetrainLeftFrontTalonFX = 14;
        public static final int kDrivetrainLeftBackTalonFX = 15;       
    }
}
