// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.coaxial.wpilibmethod;



/*
https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot
 */

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AS5600;

public class SwerveModule {
    private static final double kWheelRadius = 0.0508; // meters TODO: Change
    private static final int kEncoderResolution = 4096; //TODO: Change
    private static final double DRIVE_MOTOR_MAX_RPM = 300; //TODO: Change this
    private static final double GEAR_RATIO = 1.0;


    private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = FastMath.toRadians(60); //TODO: Change



    private final DcMotorEx m_driveMotor;
    private final CRServo m_turningServo;
    private final AS5600 sensor;

    // Gains are for example purposes only - must be determined for your own robot!
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0); //TODO: Change

    // Gains are for example purposes only - must be determined for your own robot!
    private final ProfiledPIDController m_turningPIDController =
            new ProfiledPIDController(
                    1,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(
                            kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));



    //My initial values are guesses, although the drive motor initial values are from rr.
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.002, 1.0 / rpmToVelocity(DRIVE_MOTOR_MAX_RPM));
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.002, 0.5);


    public SwerveModule(HardwareMap hardwareMap, String motorName, String servoName, String sensorName) {

        m_driveMotor = hardwareMap.get(DcMotorEx.class, motorName);
        m_turningServo = hardwareMap.get(CRServo.class, servoName);
        sensor = hardwareMap.get(AS5600.class, sensorName);


        m_driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sets the rpm fraction from 85% to 100%
        MotorConfigurationType type = m_driveMotor.getMotorType().clone();
        type.setAchieveableMaxRPMFraction(1.0);
        m_driveMotor.setMotorType(type);

    }

    /**
     * Returns the current wheel velocity in meters/sec.
     * @return Wheel velocity in m/s
     */
    public double getMotorSpeedMeters(){
        return m_driveMotor.getVelocity() / kEncoderResolution * (2.0 * Math.PI *kWheelRadius);
    }

    /**
     * Converts the given velocity in m/s to an encoder velocity in ticks/sec
     * @param meters Velocity in m/s
     * @return Velocity in ticks/sec
     */
    public double metersPerSecondToTicks(double meters){
        return meters / (2.0 * Math.PI * kWheelRadius) * kEncoderResolution;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getMotorSpeedMeters(), new Rotation2d(sensor.getAngle(AngleUnit.RADIANS))); //TODO: needs to be in radians
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        //Read the angle of the module once; it's an expensive i2c operation
        double angle = sensor.getAngle(AngleUnit.RADIANS);

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = optimize(desiredState, new Rotation2d(angle));

        // Calculate the PID drive output in encoder velocity ticks/sec
        final double driveOutput =
                m_drivePIDController.calculate(getMotorSpeedMeters(), metersPerSecondToTicks(state.speedMetersPerSecond));

        //Find the FF output for the wheel
        final double driveFeedforward =
                m_driveFeedforward.calculate(metersPerSecondToTicks(state.speedMetersPerSecond));

        // Calculate the turning cr servo output from the turning PID controller.
        final double turnOutput =
                m_turningPIDController.calculate(angle, state.angle.getRadians());
        
        //Find the FF output for the cr servo
        final double turnFeedforward =
                m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);


        m_driveMotor.setVelocity(driveOutput + driveFeedforward);
        m_turningServo.setPower(turnOutput + turnFeedforward);
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. If this is used with the PIDController class's
     * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return Optimized swerve module state.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        Rotation2d delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * kWheelRadius / 60.0;
    }

}