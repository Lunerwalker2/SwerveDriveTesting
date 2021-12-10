package org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.SwerveDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.*
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.util.LynxModuleUtil
import kotlin.collections.ArrayList

import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.MAX_ACCEL
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.MAX_ANG_ACCEL
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.MAX_ANG_VEL
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.MAX_VEL
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.MOTOR_VELO_PID
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.RUN_USING_ENCODER
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.encoderTicksToInches
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.kStatic
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.drive.DriveConstants.kV
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.coaxial.rrmethod.trajectorysequence.TrajectorySequenceRunner


class SampleSwerveDrive(private val hardwareMap: HardwareMap) : SwerveDrive(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH) {

    data class SwerveModule(val motor: DcMotorEx, val servo: Servo)
    @JvmField
    var TRANSLATIONAL_PID = PIDCoefficients(0.0, 0.0, 0.0)
    @JvmField
    var HEADING_PID = PIDCoefficients(0.0, 0.0, 0.0)

    private val trajectorySequenceRunner: TrajectorySequenceRunner

    companion object {
        private val VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH)
        private val ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL)

        fun getVelocityConstraint(maxVel: Double, maxAngularVel: Double, trackWidth: Double): TrajectoryVelocityConstraint {
            return MinVelocityConstraint(listOf(
                    AngularVelocityConstraint(maxAngularVel),
                    MecanumVelocityConstraint(maxVel, trackWidth)
            ))
        }

        fun getAccelerationConstraint(maxAccel: Double): TrajectoryAccelerationConstraint {
            return ProfileAccelerationConstraint(maxAccel)
        }
    }

    private val follower: TrajectoryFollower

    private val imu: BNO055IMU

    private val leftFront: SwerveModule
    private val leftRear: SwerveModule
    private val rightRear: SwerveModule
    private val rightFront: SwerveModule

    private val batteryVoltageSensor: VoltageSensor

    private val modules: List<SwerveModule>

    init {

        follower = HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5)

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        leftFront = SwerveModule(
                hardwareMap[DcMotorEx::class.java, "LeftFrontM"],
                hardwareMap[Servo::class.java, "LeftFrontS"]
        )
        leftRear = SwerveModule(
                hardwareMap[DcMotorEx::class.java, "LeftRearM"],
                hardwareMap[Servo::class.java, "leftRearS"]
        )
        rightRear = SwerveModule(
                hardwareMap[DcMotorEx::class.java, "RightFrontM"],
                hardwareMap[Servo::class.java, "RightRearS"]
        )
        rightFront = SwerveModule(
                hardwareMap[DcMotorEx::class.java, "RightFrontM"],
                hardwareMap[Servo::class.java, "RightFrontS"]
        )

        //Set the servos to initial positions
        leftFront.servo.position = 0.5
        leftRear.servo.position = 0.5
        rightFront.servo.position = 0.5
        rightRear.servo.position = 0.5

        modules = listOf(leftFront, leftRear, rightRear, rightFront)

        for (module in modules) {
            val motorConfigurationType = module.motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            module.motor.motorType = motorConfigurationType
        }

        if (RUN_USING_ENCODER) {
            setMode(RunMode.RUN_USING_ENCODER)
        }

        setZeroPowerBehavior(ZeroPowerBehavior.BRAKE)

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID)
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        trajectorySequenceRunner = TrajectorySequenceRunner(follower, HEADING_PID)

    }

    fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, false,  VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectoryBuilder(startPose: Pose2d, startHeading: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT)
    }

    fun trajectorySequenceBuilder(startPose: Pose2d): TrajectorySequenceBuilder {
        return TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        )
    }


    fun turnAsync(angle: Double) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(poseEstimate)
                        .turn(angle)
                        .build()
        )
    }

    fun turn(angle: Double) {
        turnAsync(angle)
        waitForIdle()
    }

    fun followTrajectoryAsync(trajectory: Trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        )
    }

    fun followTrajectory(trajectory: Trajectory) {
        followTrajectoryAsync(trajectory)
        waitForIdle()
    }

    fun followTrajectorySequenceAsync(trajectorySequence: TrajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence)
    }

    fun followTrajectorySequence(trajectorySequence: TrajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence)
        waitForIdle()
    }

    fun getLastError(): Pose2d {
        return trajectorySequenceRunner.lastPoseError
    }

    fun update() {
        updatePoseEstimate()
        val signal = trajectorySequenceRunner.update(poseEstimate, poseVelocity)
        if(signal != null) setDriveSignal(signal)
    }

    fun waitForIdle() {
        while (!Thread.currentThread().isInterrupted && isBusy()) update()
    }

    fun isBusy(): Boolean {
        return trajectorySequenceRunner.isBusy
    }

    fun setMode(runMode: RunMode) {
        for (module in modules) {
            module.motor.mode = runMode
        }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: ZeroPowerBehavior) {
        for (module in modules) {
            module.motor.zeroPowerBehavior = zeroPowerBehavior
        }
    }

    fun setPIDFCoefficients(runMode: RunMode, coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.voltage
        )
        for (module in modules) {
            module.motor.setPIDFCoefficients(runMode, compensatedCoefficients)
        }
    }

    override fun setMotorPowers(v: Double, v1: Double, v2: Double, v3: Double) {
        leftFront.motor.power = v
        leftRear.motor.power = v1
        rightRear.motor.power = v2
        rightFront.motor.power = v3
    }

    override fun setModuleOrientations(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        val orientations = listOf(frontLeft, rearLeft, rearRight, frontRight)
        for(i in 0..3){
            modules[i].servo.position = orientations[i]
        }
    }

    override fun getModuleOrientations(): List<Double> {
        val moduleOrientations = ArrayList<Double>()
        //Doesn't actually return the current position, just the last commanded position
        for(module in modules){
            moduleOrientations.add(module.servo.position)
        }
        return moduleOrientations
    }

    override fun getWheelPositions(): List<Double> {
        val wheelPositions: MutableList<Double> = ArrayList()
        for (module in modules) {
            wheelPositions.add(encoderTicksToInches(module.motor.currentPosition.toDouble()))
        }
        return wheelPositions
    }

    override fun getWheelVelocities(): List<Double> {
        val wheelVelocities: MutableList<Double> = ArrayList()
        for (module in modules) {
            wheelVelocities.add(encoderTicksToInches(module.motor.velocity))
        }
        return wheelVelocities
    }

    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()


    override fun getExternalHeadingVelocity(): Double {

        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface
        return -imu.angularVelocity.xRotationRate.toDouble()
    }


}