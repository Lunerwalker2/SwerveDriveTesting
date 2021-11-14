package com.circuitrunners;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/**
 * Just everything in one place. I am so very sorry for this.
 */
public class Robot {

    private Intake intake;
    private Launcher launcher;
    private WobbleGoalArm goalArm;
    private SwerveDrive driveTrain;
    private OpenCvCamera webcam;

    private StartStackPipeline startStackPipeline;

    private boolean isAuto;

    public Robot(OpMode opmode, boolean auto) {
        // The next few lines just create the various subsystems with the mapped hardware.
        //intake = new Intake(opmode.hardwareMap.get(Servo.class, "leftServo"), opmode.hardwareMap.get(Servo.class, "rightServo"));
        //launcher = new Launcher(opmode.hardwareMap.get(DcMotor.class, "launcherMotor"));
        //goalArm = new WobbleGoalArm(opmode.hardwareMap.get(Servo.class, "armServo"), opmode.hardwareMap.get(Servo.class, "clawServo"));
        driveTrain = new SwerveDrive(new SwerveWheel(opmode.hardwareMap.get(DcMotor.class, "rightFrontMotor"),
                opmode.hardwareMap.get(Servo.class, "rightFrontServo")),
                new SwerveWheel(opmode.hardwareMap.get(DcMotor.class, "leftFrontMotor"),
                        opmode.hardwareMap.get(Servo.class, "leftFrontServo")),
                new SwerveWheel(opmode.hardwareMap.get(DcMotor.class, "leftBackMotor"),
                        opmode.hardwareMap.get(Servo.class, "leftBackServo")),
                new SwerveWheel(opmode.hardwareMap.get(DcMotor.class, "rightBackMotor"),
                        opmode.hardwareMap.get(Servo.class, "rightBackServo")), 1, 1,
                opmode.hardwareMap.get(BNO055IMU.class, "swerveIMU"));
          //

        // Creates an instance of a webcam with the hardware map
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(opmode.hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Sets up the webcam with the pipeline that tells the camera what do with its frames
        //startStackPipeline = new StartStackPipeline();
        //webcam.setPipeline(startStackPipeline);

        // Async is short for asynchronous, meaning the webcam does what it needs to separately from everything else
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            // This is essentially a function passed as an argument for the camera listener
//            @Override
//            public void onOpened() { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }
//        });

        isAuto = auto;
    }

    public void drive(double moveX, double moveY, double turnX, double turnY) throws Exception {
        if (isAuto) throw new Exception("The robot cannot drive normally in the autonomous period!");
        driveTrain.driveRobot(moveX, moveY, turnX, turnY);
    }

    public void driveAuto(double aSpeed, double aAngle, double aOrient, double aDist) {
        driveTrain.autoDrive(aSpeed, aAngle, aOrient, aDist);
    }

    public void setLauncher(Launcher.LaunchPower power) {
        launcher.setPower(power);
    }

    public void setIntake(double pos) {
        intake.setIntakePos(pos);
    }

    public void setIntakeDelta(double delta) {
        intake.setIntakePosDelta(delta);
    }

    public void startLaunching() { launcher.startLaunching(); }

    public void stopLaunching() { launcher.stopLaunching(); }

    public void toggleGoalArm() { goalArm.toggleArm(); }

    public void toggleGoalClaw() { goalArm.toggleClaw(); }

    public StartStackPipeline.RingState getRingState() {
        return startStackPipeline.getState();
    }
}
