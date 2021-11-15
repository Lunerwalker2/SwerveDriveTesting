package org.firstinspires.ftc.teamcode.drive

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

class AbsoluteEncoderOrientationSensor(private val hardwareMap: HardwareMap, private val name: String) : ModuleOrientationSensor {

    private val analogSensor: AnalogInput

    init {
        analogSensor = hardwareMap[AnalogInput::class.java, name]
    }


    override fun getModuleOrientation(): Double {
        return Math.toRadians(AngleUnit.normalizeDegrees(Range.scale(
                analogSensor.voltage,
                0.0,
                analogSensor.maxVoltage,
                0.0,
                360.0
        )))
    }

}