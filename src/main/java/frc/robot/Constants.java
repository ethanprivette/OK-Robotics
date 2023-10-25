// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/** Add your docs here. */
public final class Constants {

    public static final int LEFT_SPARK_PWM = 1;
    public static final int RIGHT_SPARK_PWM = 2;
    
    public static final double HD_HEX_MAX_RPM = 6000;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.381;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4064;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = HD_HEX_MAX_RPM / 60.0 *
        //drive reduction *
        0.0762 * Math.PI;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
            DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(DRIVETRAIN_TRACKWIDTH_METERS);

}
