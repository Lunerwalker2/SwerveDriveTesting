package org.firstinspires.ftc.teamcode.onlinetutorial;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AS5600;

@Disabled
@TeleOp
public class TutorialSwerveTeleOp extends LinearOpMode {

    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;

    private SwerveDrive swerveDrive;

    @Override
    public void runOpMode() throws InterruptedException {

        backRight = new WheelDrive(
                hardwareMap.get(DcMotorEx.class, "backRightMotor"),
                hardwareMap.get(CRServo.class, "backRightServo"),
                hardwareMap.get(AS5600.class, "backRightSensor")
        );
        backLeft = new WheelDrive(
                hardwareMap.get(DcMotorEx.class, "backLeftMotor"),
                hardwareMap.get(CRServo.class, "backLeftServo"),
                hardwareMap.get(AS5600.class, "backLeftSensor")
        );
        frontRight = new WheelDrive(
                hardwareMap.get(DcMotorEx.class, "frontRightMotor"),
                hardwareMap.get(CRServo.class, "frontRightServo"),
                hardwareMap.get(AS5600.class, "frontRightSensor")
        );
        frontLeft = new WheelDrive(
                hardwareMap.get(DcMotorEx.class, "frontLeftMotor"),
                hardwareMap.get(CRServo.class, "frontLeftServo"),
                hardwareMap.get(AS5600.class, "frontLeftSensor")
        );

        swerveDrive = new SwerveDrive(backRight, backLeft, frontRight, frontLeft);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){
            //reverse left stick y
            swerveDrive.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        //stop when done
        swerveDrive.drive(0,0,0);


    }
}
