// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    // Xbox Controllers Port Indexes
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int TEST_CONTROLLER_PORT = 2;

    public static final int PDP_CAN_ID = 0;

    // Sensors
    public static final int INTAKE_BEAM_BREAK = 0;

    // Drive Motors
    public static final int LEFT_FRONT_MOTOR = 4;
    public static final int LEFT_BACK_MOTOR = 1;
    public static final int RIGHT_FRONT_MOTOR = 2;
    public static final int RIGHT_BACK_MOTOR = 3;

    public static final int CLIMBER_WENCH_MOTOR = 6;

    public static final int INTAKE_MOTOR = 5;

    public static final int ARM_MOTOR_LEFT = 7;
    public static final int ARM_MOTOR_RIGHT = 8;

    public static final int SHOOTER_LEFT = 10;
    public static final int SHOOTER_RIGHT = 11;

    public static final double ARM_Offset = Units.degreesToRadians(319.0 - 180.0);

    // Radios
    public static final double INTAKE_RADIO = (1.0/3.2727);
    public static final double SHOOTER_RADIO = (1.0/1.0);
    public static final double WENCH_RADIO = (1.0/48.0);
    public static final double ARM_RADIO = (1.0/1.0);
    public static final double DRIVE_RADIO = (1.0/8.0);



    public static final class SwerveConstants
    {
        public static final double stickDeadband = 0.07;

        public static final int pigeonID = 21;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.5);
        public static final double wheelBase = Units.inchesToMeters(21.5);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;
    
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
    
        public static final double driveGearRatio = (8.14 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1
        

        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 40;
            
        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;
        
            /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor =
            (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxSpeed = 0.5; // meters per second, changed to 0.5 (from 3.0) for testing/demonstration purposes 
        public static final double maxAngularVelocity = 3.0; // changed to 6.0 (from 11.5) for testing/demonstration purposes

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = true;
        public static final boolean angleInvert = false;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;


        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
        public static final int driveMotorID = 7;
        public static final int angleMotorID = 8;
        public static final int canCoderID = 16;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-145.020);
        //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(145.722656);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 13;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-12.568);
        //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(13.0078125);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
        public static final int driveMotorID = 5;
        public static final int angleMotorID = 6;
        public static final int canCoderID = 15;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-273.164);
        // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-87.01172);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
        public static final int driveMotorID = 3;
        public static final int angleMotorID = 4;
        public static final int canCoderID = 14;
        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-354.990);
        //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-4.921875);
        public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}