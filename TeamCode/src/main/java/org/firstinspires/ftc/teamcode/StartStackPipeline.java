package com.circuitrunners;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This class checks the height of the starting stack for use in changing auto behavior.
 */
public class StartStackPipeline extends OpenCvPipeline {

    private final Scalar RED = new Scalar(255, 0, 0);
    private final Scalar GREEN = new Scalar(0, 255, 0);

    boolean paused = false;

    private RingState state;

    public enum RingState {
        ZERO_RINGS,
        ONE_RING,
        FOUR_RINGS
    }

    /**
     * This processes every frame the camera sees.
     * @param input The current frame
     * @return The frame after any reading and/or manipulation
     */
    @Override
    public Mat processFrame(Mat input) {

        // The commented lines deal with drawing on the frame for visualization. Only uncomment if we would like visuals.

        double[] topCheckPoint = input.get(input.width() / 2, input.height() / 4);
        double[] bottomCheck = input.get(input.width() / 2, input.height() / 4 * 3);

        boolean isTopYellow = false;
        boolean isBottomYellow = false;

        if ((bottomCheck[0] + bottomCheck[1]) / 2 > 128) {
            isBottomYellow = true;
            if ((topCheckPoint[0] + topCheckPoint[1]) / 2 > 128) {
                isTopYellow = true;
                state = RingState.FOUR_RINGS;
            }
            else {
                state = RingState.ONE_RING;
            }
        }
        else {
            state = RingState.ZERO_RINGS;
        }

        if (!paused) {
            Imgproc.circle(input, new Point(input.width() / 2, input.height() / 4), 4, (isTopYellow) ? GREEN : RED);
            Imgproc.circle(input, new Point(input.width() / 2, input.height() / 4 * 3), 4, (isBottomYellow) ? GREEN : RED);
        }

        return input;
    }

    /**
     * Gets the start stack's determined height for the current frame.
     * @return The state of the start stack
     */
    public RingState getState() {
        return state;
    }

    public void toggleVisuals() {
        paused = !paused;
    }
}
