package org.firstinspires.ftc.teamcode.onlinetutorial;

import org.apache.commons.math3.util.FastMath;

/*
https://jacobmisirian.gitbooks.io/frc-swerve-drive-programming/content/chapter1.html

Maybe it'll work who knows.

 */
public class SwerveDrive {

    //Front to back wheels in
    public static double LENGTH = 16;

    //Left to right wheels in
    public static double WIDTH = 16;

    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;

    public SwerveDrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
    }


    /*
    x1 = translation x
    y1 = translation y
    x2 = rotation x
     */
    public void drive(double x1, double y1, double x2){

        double r = FastMath.hypot(LENGTH, WIDTH);

        y1 *= -1;

        double a = x1 - x2 * (LENGTH / r);
        double b = x1 + x2 * (LENGTH / r);
        double c = y1 - x2 * (WIDTH / r);
        double d = y1 + x2 * (WIDTH / r);

        //Calculate motor speeds, range 0-1
        double backRightSpeed = FastMath.hypot(a, d);
        double backLeftSpeed = FastMath.hypot(a, c);
        double frontRightSpeed = FastMath.hypot(b, d);
        double frontLeftSpeed = FastMath.hypot(b, c);

        //Calculate wheel angles in range of -1 to 1
        double backRightAngle = FastMath.atan2(a, d) / FastMath.PI;
        double backLeftAngle = FastMath.atan2(a, c) / FastMath.PI;
        double frontRightAngle = FastMath.atan2(b, d) / FastMath.PI;
        double frontLeftAngle = FastMath.atan2(b, c) / FastMath.PI;

        backRight.drive (backRightSpeed, backRightAngle);
        backLeft.drive (backLeftSpeed, backLeftAngle);
        frontRight.drive (frontRightSpeed, frontRightAngle);
        frontLeft.drive (frontLeftSpeed, frontLeftAngle);
    }
}
