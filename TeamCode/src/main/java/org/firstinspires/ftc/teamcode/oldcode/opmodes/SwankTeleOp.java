package org.firstinspires.ftc.teamcode.oldcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.oldcode.SwerveCore;


@TeleOp(name="SwankTeleOp",group="Swerve")
public class SwankTeleOp extends SwerveCore {
    double leftMove, rightMove;
    double leftPos,rightPos;
    boolean debounce1 = false;
    boolean debounce2 = false;
    boolean closed = false;
    boolean armup=true;
    public SwankTeleOp() {

    }

    public void init() {
        swerveDebug(500, "SwerveTeleOp::init", "START");

        // Run initialization of other parts of the class
        // Note that the class will connect to all of our motors and servos
        super.init();

        goalArm.setPosition(.5);
        goalClaw.setPosition(0);


        swerveDebug(500, "SwerveTeleOp::init", "DONE");
    }


    @Override
    public void start() {
        swerveDebug(500, "SwerveTeleOp::start", "START");

        // Call the super/base class start method.
        super.start();

        swerveDebug(500, "SwerveTeleOp::start", "DONE");
    }
    public void loop(){
        leftMove = -gamepad1.left_stick_y;
        rightMove = -gamepad1.right_stick_x;
        telemetry.addData("IntakeEncAverage",(leftIntake.getCurrentPosition()+rightIntake.getCurrentPosition())/2);
        ourSwerve.swankDriveWheels(leftMove,rightMove);

        wobbleGoal(gamepad2.a,gamepad2.x);
        intake(gamepad2.right_trigger,gamepad2.left_trigger);
        if(gamepad2.right_bumper)
            intServo.setPower(-1);
        else if(gamepad2.left_bumper)
            intServo.setPower(1);
        else
            intServo.setPower(0);


    }


    private void wobbleGoal(boolean a, boolean x){
        if(a&&debounce1==false){
            toggleArm();
            debounce1=true;
        }else if(!a){
            debounce1=false;
        }
        if(x&&debounce2==false){
            toggleClaw();
            debounce2=true;
        }else if(!x){
            debounce2=false;
        }

    }
    private void intake(double up, double down) {
        double upPower = up * .6;
        double downPower = down * .75;
//        if (down <.1 && up <.1){
//            leftIntake.setPower(.4);
//            rightIntake.setPower(.4);
//        }
        if (down < .1) {
            leftIntake.setPower(upPower);
            rightIntake.setPower(upPower);

        } else if (down > .1) {
            leftIntake.setPower(-downPower);
            rightIntake.setPower(-downPower);
        }
        else{
            leftIntake.setPower(.3);
            rightIntake.setPower(.3);
        }
    }
    private void toggleArm(){
        if(armup)
            goalArm.setPosition(.2);
        else
            goalArm.setPosition(.5);
        armup=!armup;
    }
    private void toggleClaw(){
        if(closed)
            goalClaw.setPosition(.55);
        else

            goalClaw.setPosition(0);
        closed=!closed;
    }


}
