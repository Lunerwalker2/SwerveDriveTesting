package org.firstinspires.ftc.teamcode.onlinetutorial;



import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.AS5600;

/*
This is where we differ from this guide a little, as in the ftc bot we obviously don't
have two motors per module, rather one motor for the wheel and one servo.

TODO: I have noticed that this does lack the servo flipping features of the original swerve drive,
but if this method turns out to be the best one, then that can be focused on a lot more.
 */
public class WheelDrive {

    public static PIDCoefficients MODULE_HEADING_PID = new PIDCoefficients(0.0, 0.0, 0.0);

    private DcMotorEx speedMotor;
    private CRServo angleServo;

    //Might be what we actually use; a stand in at the very least.
    private AS5600 angleSensor;


    /*
    Controller for the module angle. The guide uses the wpilib controller, but as this is not
    wpilib, we don't have that specific class. While we could use the FTCLib PIDController which is
    quite similar to the wpilib one, we are going to use the RR controller as it is an extremely good
    implementation.

    This will require some different implementations of things later on.
     */
    private PIDFController angleController;

    public WheelDrive(DcMotorEx speedMotor, CRServo angleServo, AS5600 angleSensor){
        this.speedMotor = speedMotor;
        this.angleServo = angleServo;
        this.angleSensor = angleSensor;

        //In wpilib you can just make this work in the background, LMAO LOL WELCOME TO FTC NOOB
        angleController = new PIDFController(MODULE_HEADING_PID);
        angleController.setOutputBounds(-1, 1);
    }


    /*
     * Drive method as defined in guide, with a few things. First, we need to update the
     * PID controller here, although ideally this would be in an update method and this
     * drive method would only be for "setting" purposes, as having your PID controller update
     * like this seems like bad practice.
     *
     * Secondly, this has 0 servo flipping optimization as said above. THIS IS BAD. But its a proof
     * of concept more than anything right now.
     *
     * speed is 0 to 1
     * angle is -1 to 0
     */
    public void drive(double speed, double angle){
        speedMotor.setPower(speed);

        angleController.setTargetPosition(angle * 180.0); //convert to euler angles
        angleServo.setPower(angleController.update(angleSensor.getAngle())); //TODO: FIX ACTUAL SENSOR

    }

}
