package com.circuitrunners;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * This class handles the intake.
 */
public class Intake {

    private TestServo leftMotor, rightMotor;

    /**
     * Creates a new instance of this with two regular servos
     * @param aLeftMotor The left servo of the intake
     * @param aRightMotor The right servo of the intake
     */
    public Intake(Servo aLeftMotor, Servo aRightMotor) {
        leftMotor = new TestServo(aLeftMotor, false, "leftMotor");
        rightMotor = new TestServo(aRightMotor, false, "rightMotor");
    }

    /**
     * Creates a new instance of this with two TestServo instances
     * @param aLeftMotor The left servo of the intake
     * @param aRightMotor The right servo of the intake
     */
    public Intake(TestServo aLeftMotor, TestServo aRightMotor) { // see above comment
        leftMotor = aLeftMotor;
        rightMotor = aRightMotor;
    }

    /**
     * Sets the intake to a certain position.
     * @param pos The position to set the intake bar to
     */
    public void setIntakePos(double pos) {
        leftMotor.setPosition(pos);
        rightMotor.setPosition(pos);
    }

    /**
     * Changes the intake's position by a certain amount.
     * @param amount The amount to change the intake bar by
     */
    public void setIntakePosDelta(double amount) {
        leftMotor.setPosition(leftMotor.getTarget() + amount);
        rightMotor.setPosition(rightMotor.getTarget() + amount);
    }

    /**
     * Changes the intake's position to a specific preset launch power for use in actual competition
     * @param power The launch power to use. See LaunchPower for more details.
     * @throws IllegalArgumentException If the argument provided is not a sanctioned launch power, the code will tell you <em>exactly</em> how you messed up.
     */
    public void setIntakePosLeveled(LaunchPower power) throws IllegalArgumentException { // TODO: Figure out the numbers for this one
        switch (power) {
            case MED_GOAL: break;
            case HIGH_GOAL: break;
            case POWER_SHOT: break;
            default: throw new IllegalArgumentException("You have to enter a LaunchPower into this method, not a raw value");
        }
    }

    /**
     * Returns the position of the intake
     * @return The current position of the intake
     */
    public double getIntakePos() {
        return (leftMotor.getTarget() + rightMotor.getTarget()) / 2;
    }
}
