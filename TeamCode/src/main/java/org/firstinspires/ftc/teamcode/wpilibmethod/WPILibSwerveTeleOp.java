// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.firstinspires.ftc.teamcode.wpilibmethod;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervebot
 */
@Disabled
@TeleOp
public class WPILibSwerveTeleOp extends LinearOpMode {


    private Drivetrain m_swerve;

  /*
  This was in the wpilib example for autonomous. We aren't doing auto yet, but
  in the future this may be important

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

   */

    @Override
    public void runOpMode() throws InterruptedException {
        m_swerve = new Drivetrain(hardwareMap);

        if(isStopRequested()) return;

        while (opModeIsActive()) {
            driveWithJoystick(true);
        }
    }

    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.

        //Refer to above comments for explanations (ryan note)
        m_swerve.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, fieldRelative);
    }
}