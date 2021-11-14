package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * The arm meant to manipulate the wobble goal. Only toggleable.
 */
public class WobbleGoalArm {

    private TestServo armServo;
    private TestServo clawServo;

    /**
     * Creates a wobble goal arm with two TestServos
     * @param anArmServo the servo that moves the arm up and down
     * @param aClawServo the servo that closes the claw
     */
    public WobbleGoalArm(TestServo anArmServo, TestServo aClawServo) {
        armServo = anArmServo;
        clawServo = aClawServo;
    }

    /**
     * Creates a wobble goal arm with two servos
     * @param anArmServo the servo that moves the arm up and down
     * @param aClawServo the servo that closes the claw
     */
    public WobbleGoalArm(Servo anArmServo, Servo aClawServo) {
        this(new TestServo(anArmServo, false, "armServo"),
                new TestServo(aClawServo, false, "clawServo"));
    }

    /**
     * Toggles the arm's position from up to down or vice-versa.
     */
    public void toggleArm() {
    } // TODO: figure out the numbers for this

    /**
     * Toggles the claw from being open to closed or vice versa.
     */
    public void toggleClaw() {
    } // TODO: figure out the numbers for this
}
