// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.wpilibmethod;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/*
https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot
 */

public class Drivetrain {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = FastMath.toRadians(130); // 1/2 rotation per second

    //meters
    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;

    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private BNO055IMU imu;

    private final SwerveDriveKinematics m_kinematics =
            new SwerveDriveKinematics(
                    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final SwerveDriveOdometry m_odometry;

    public double getRobotHeadingRad(){
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle; //TODO: Change this
    }


    public Drivetrain(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d());
    }

    public Drivetrain(HardwareMap hardwareMap, Pose2d initialPose){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        m_frontLeft = new SwerveModule(hardwareMap, "frontLeftMotor", "frontLeftServo", "frontLeftSensor");
        m_frontRight = new SwerveModule(hardwareMap, "frontRightMotor", "frontRightServo", "frontRightSensor");
        m_backLeft = new SwerveModule(hardwareMap, "backLeftMotor", "backLeftServo", "backLeftSensor");
        m_backRight = new SwerveModule(hardwareMap, "backRightMotor", "backRightServo", "backRightSensor");

        timer.reset();
        m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(getRobotHeadingRad()), initialPose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
                m_kinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, new Rotation2d(getRobotHeadingRad()))
                                : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Updates the field relative position of the robot.
     */
    public void updateOdometry() {

        m_odometry.updateWithTime(
                timer.seconds(),
                new Rotation2d(getRobotHeadingRad()),
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_backLeft.getState(),
                m_backRight.getState());
    }
}