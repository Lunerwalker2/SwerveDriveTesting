package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


//@I2cDeviceType
//@DeviceProperties(
//        name = "AS5600 Magnetic Rotary Sensor",
//        description = "A magnetic sensor that tracks heading.",
//        xmlTag = "AS5600",
//        compatibleControlSystems = ControlSystem.REV_HUB
//)
public class AS5600 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    //All user-intractable registers
    /*
    Some registers have two bytes
    They have HI and LOW parts to them. The number is spread across both
    i.e. 0000 0000 0000 0000 = HI{0000 0000} LOW{0000 0000}
     */
    public enum Register {

        //Configuration Registers
        FIRST(0),
        ZMCO(0x00),
        ZPOS_HI(0x01),
        ZPOS_LOW(0x02),
        MPOS_HI(0x03),
        MPOS_LOW(0x04),
        MANG_HI(0x05),
        MANG_LOW(0x06),
        CONF_HI(0x07),
        CONF_LOW(0x08),

        //Output Registers
        RAW_ANGLE_HI(0x0c),
        RAW_ANGLE_LOW(0x0d),
        ANGLE_HI(0x0E),
        ANGLE_LOW(0x0F),

        //Status Registers
        STATUS(0x0b),
        AGC(0x1a),
        MAGNITUDE_HI(0x1b),
        MAGNITUDE_LOW(0x1c),

        //Burn Commands
        BURN(0xff),

        //Read Window things
        LAST(BURN.bVal);

        public int bVal;

        Register(int bVal){
            this.bVal = bVal;
        }

    }

    /**
     * returns angle in the specified unit in euler angles, i.e, -180 to 180 or -pi to pi.
     *
     * PLACEHOLDER OF 23 RN
     */
    public double getAngle(AngleUnit unit){
        short data = getRawAngle();

        return unit.fromDegrees(data);
    }

    /**
     * Returns the scaled angle (the ANGLE reg as opposed to the RAW_ANGLE reg) in degrees.
     *
     * From the factory the range is 0 to 360 with a 10-LSB hysteresis between 0 and 360.
     * If the range has been changed, this output will follow that instead.
     * @return Raw angle reading degrees
     */
    public short getRawAngle(){
        short raw = readShort(Register.ANGLE_HI);

        //number is 12 bit, so mask the top 4 bits to 0s
        raw &= 0x0fff;
        return raw;
    }

    protected void writeShort(Register reg, short data){
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(data));
    }

    protected short readShort(Register reg){
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    protected byte read8(final Register reg){
        return deviceClient.read8(reg.bVal);
    }

    protected void write8(final Register reg, final int data){
        this.deviceClient.write8(reg.bVal, data);
    }


    @Override
    public Manufacturer getManufacturer(){
        return Manufacturer.AMS;
    }

    @Override
    public synchronized boolean doInitialize(){
        return true;
    }

    @Override
    public String getDeviceName(){
        return "AS5600 Magnetic Rotary Sensor";
    }

    //Constructor to initialize device
    public AS5600(I2cDeviceSynch deviceClient){
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x36));

        super.registerArmingStateCallback(false); //Deals with the USB getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();


    }

}