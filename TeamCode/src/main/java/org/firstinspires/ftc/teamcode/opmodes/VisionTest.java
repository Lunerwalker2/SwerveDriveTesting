package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.StartStackPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class VisionTest extends OpMode {
    OpenCvCamera webcam;
    StartStackPipeline startStackPipeline;
    Gamepad gamepad;

    @Override
    public void init() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(this.hardwareMap.get(WebcamName.class, "vision_camera"), cameraMonitorViewId);

        startStackPipeline = new StartStackPipeline();
        webcam.setPipeline(startStackPipeline);

        webcam.openCameraDevice();

        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        gamepad = hardwareMap.get(Gamepad.class, "gamepad1");
    }

    public void loop() {
        if (gamepad.b) {
            webcam.stopStreaming();
            this.stop();
        }

        else if (gamepad.a) {
            startStackPipeline.toggleVisuals();
        }
    }
}
