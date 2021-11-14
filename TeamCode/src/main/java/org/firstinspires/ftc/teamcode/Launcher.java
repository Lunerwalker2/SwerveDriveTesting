package com.circuitrunners;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This class handles the ring launcher.
 */
public class Launcher {

    private TestMotor launcherMotor;
    private double launchPower = 0.5;

    /**
     * The enum that represents the target of the launcher.
     */


    /**
     * Creates a launcher with one DcMotor
     * @param aLauncherMotor the motor that spins the flywheel
     */
    public Launcher(DcMotor aLauncherMotor) {
        this(new TestMotor(aLauncherMotor, "launcherMotor"));
    }

    /**
     * Creates a Launcher with one TestMotor
     * @param aLauncherMotor the motor that spins the flywheel
     */
    public Launcher(TestMotor aLauncherMotor) {
        launcherMotor = aLauncherMotor;
    }

    /**
     * Sets the launcher's power to one of the three presets, switching between the
     * @param power The LaunchPower to launch any rings at
     */
    public void setPower(LaunchPower power) throws IllegalArgumentException { // TODO: figure out the numbers for this.
        switch (power) {
            case MED_GOAL: break;
            case HIGH_GOAL: break;
            case POWER_SHOT: break;
            default: throw new IllegalArgumentException("You have to enter a LaunchPower into this method, not a raw value");
        }
    }

    public void startLaunching() { launcherMotor.setSpeed(launchPower); }

    public void stopLaunching() { launcherMotor.setSpeed(0); }

}
