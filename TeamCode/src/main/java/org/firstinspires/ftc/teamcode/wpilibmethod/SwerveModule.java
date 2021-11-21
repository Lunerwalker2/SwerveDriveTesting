// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.wpilibmethod;



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

    public double getMotorSpeedMeters(){
        return m_driveMotor.getVelocity(AngleUnit.RADIANS) * 2 * Math.PI * kWheelRadius / kEncoderResolution;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveMotor.getVelocity(AngleUnit.DEGREES), new Rotation2d(sensor.getAngle())); //TODO: needs to be in radians
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degreesg
        SwerveModuleState state = optimize(desiredState, new Rotation2d(sensor.getAngle()));

        // Calculate the drive output in encoder velocity ticks/sec
        final double driveOutput = state.speedMetersPerSecond / (2 * Math.PI * kWheelRadius) * kEncoderResolution;

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
                m_turningPIDController.calculate(sensor.getAngle(), state.angle.getRadians());


        m_driveMotor.setVelocity(driveOutput);
        m_turningServo.setPower(turnOutput);
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
}