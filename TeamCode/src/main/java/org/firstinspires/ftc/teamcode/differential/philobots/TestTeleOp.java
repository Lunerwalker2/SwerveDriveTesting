package org.firstinspires.ftc.teamcode.differential.philobots;

import static org.firstinspires.ftc.teamcode.differential.philobots.RobotUtil.checkDeadband;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.differential.philobots.math.Vector2d;

@TeleOp(name = "Diff Swerve TeleOp", group = "TeleOp")
public class TestTeleOp extends LinearOpMode {
    Robot robot;

    public boolean willResetIMU = true;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(this, false);

        //allows driver to indicate that the IMU should not be reset
        //used when starting TeleOp after auto or if program crashes in the middle of match
        //relevant because of field-centric controls
        while (!isStarted()){
            if (gamepad1.y) {
                willResetIMU = false;
            }
        }

        if (willResetIMU) robot.initIMU();


        if(isStopRequested())return;

        while (opModeIsActive()){
            Vector2d joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
            //the y parameter here isn't used
            Vector2d joystick2 = new Vector2d(gamepad1.right_stick_x, -gamepad2.right_stick_y); //RIGHT joystick

            robot.driveController.updateUsingJoysticks(checkDeadband(joystick1), checkDeadband(joystick2));


//        //uncomment for live tuning of ROT_ADVANTAGE constant
//        if (gamepad1.b) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE += 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE += 0.01;
//        }
//        if (gamepad1.x) {
//            robot.driveController.moduleRight.ROT_ADVANTAGE -= 0.01;
//            robot.driveController.moduleLeft.ROT_ADVANTAGE -= 0.01;
//        }
//        telemetry.addData("ROT_ADVANTAGE: ", robot.driveController.moduleLeft.ROT_ADVANTAGE);


            //to confirm that joysticks are operating properly
            telemetry.addData("Joystick 1", joystick1);
            telemetry.addData("Joystick 2", joystick2);

            telemetry.update();
        }
    }
}