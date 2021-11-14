package org.firstinspires.ftc.teamcode;
/**
 * These are the three places we are planning to shoot rings into.
 * The middle goal (MED_GOAL) is only to be used if something breaks and we can't use the high goal.
 * The high goal (HIGH_GOAL) is overwhelmingly our target. The highest points per second comes from this.
 * The power shot targets (POWER_SHOT) require, as expected, more power than height.
 */
public enum LaunchPower {
    MED_GOAL,
    HIGH_GOAL,
    POWER_SHOT
}
