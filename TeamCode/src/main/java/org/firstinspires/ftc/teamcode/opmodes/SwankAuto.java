package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SwerveCore;

@Autonomous(name="SwankAuto",group="Swerve")
public class SwankAuto extends SwerveCore {
    public void init(){

        swerveDebug(500, "SwankAuto::init", "STARTing init for TETS");

        super.init();

        goalArm.setPosition(.5);
        goalClaw.setPosition(.55);

        crater = Boolean.TRUE;


        // Robot and autonomous settings are read in from files in the core class init()
        // Report the autonomous settings
        showAutonomousGoals();

        swerveLog( "X S6", ourSwerve.getOrientLog());

        swerveDebug(500, "SwankAuto::init", "DONE");

    }

    public void start() {
        super.start();

        ourSwerve.swankDriveWheels(.8, 0);
        swerveSleep(2300);
        ourSwerve.swankDriveWheels(0, 0);
        goalArm.setPosition(.2);
        goalClaw.setPosition(0);
        swerveSleep(1000);
        ourSwerve.swankDriveWheels(-.8, 0);
        swerveSleep(1000);
        ourSwerve.swankDriveWheels(0, 0);
    }


}
