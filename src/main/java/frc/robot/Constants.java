// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/** Add your docs here. */
public final class Constants {

    public static final int LEFT_TALON_PWM = 0;
    public static final int RIGHT_TALON_PWM = 0;

    public static final int LEFT_ENCODER_DIO_1 = 0;
    public static final int LEFT_ENCODER_DIO_2 = 0;

    public static final int RIGHT_ENCODER_DIO_1 = 0;
    public static final int RIGHT_ENCODER_DIO_2 = 0;
    
    public static final double HD_HEX_MAX_RPM = 6000;
    public static final double HD_HEX_ENCODER_CPR = 28.0;
    public static final double HD_HEX_COUNTS_PER_DEGREE = HD_HEX_ENCODER_CPR / 360;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.381;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4064;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = HD_HEX_MAX_RPM / 60.0 *
        //drivereduction *
        0.0762 * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(DRIVETRAIN_TRACKWIDTH_METERS);


    public static final int ELEVATOR_TALON_PWM = 0;

    public static final int ELEVATOR_ENCODER_DIO_1 = 0;
    public static final int ELEVATOR_ENCODER_DIO_2 = 0;

    public static final double ELEVATOR_GEAR_RATIO = 75.0;
    public static final double ELEVATOR_ROTATIONS_PER_DEGREE = ELEVATOR_GEAR_RATIO / 360.0;


    public static final int TURRET_TALON_PWM = 0;

    public static final int TURRET_ENCODER_DIO_1 = 0;
    public static final int TURRET_ENCODER_DIO_2 = 0;
}
