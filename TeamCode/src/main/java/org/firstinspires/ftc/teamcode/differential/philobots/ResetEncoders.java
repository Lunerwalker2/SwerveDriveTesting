package org.firstinspires.ftc.teamcode.differential.philobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//INSTRUCTIONS:
// align modules to be facing the same direction (make sure not 180 degrees apart)
// press y button on controller 1 (configured with start+A)
// make sure telemetry encoder values are both 0
// if robot controls are inverted, repeat this process, but turn both modules 180 degrees away from where you reset them before

//MUST be run every time program is downloaded
//does NOT have to be run before every TeleOp/Auto run
//probably should be used to verify that encoders have not drifted before every competition match

@TeleOp(name = "Reset Encoders", group = "Utilities")
public class ResetEncoders extends LinearOpMode {
    Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, false);

        waitForStart();

        if(isStopRequested())return;

        while (opModeIsActive()){
            telemetry.addData("LEFT Module Orientation: ", robot.driveController.moduleLeft.getCurrentOrientation().getAngle());
            telemetry.addData("RIGHT Module Orientation: ", robot.driveController.moduleRight.getCurrentOrientation().getAngle());
            telemetry.update();

            if (gamepad1.y) {
                robot.driveController.resetEncoders();
            }
        }
    }
}
