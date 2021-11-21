package org.firstinspires.ftc.teamcode.rrmethod.drive


/**
 * Sensor which can return the current orientation of a module's heading in the range of
 * -Pi to Pi.
 *
 */
interface ModuleOrientationSensor {


    /**
     * Returns the module orientation in the range of -Pi to Pi.
     */
    fun getModuleOrientation(): Double
}