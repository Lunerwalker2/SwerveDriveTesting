package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.roadrunner.drive.SwerveDrive
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.HardwareMap

class SampleSwerveDrive(private val hardwareMap: HardwareMap): SwerveDrive(

) {


    private lateinit var imu: BNO055IMU






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
        return imu.angularVelocity.zRotationRate.toDouble()
    }
}