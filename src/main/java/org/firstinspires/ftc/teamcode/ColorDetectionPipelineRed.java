package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorDetectionPipelineRed extends OpenCvPipeline {

    // Define the colors in HSV space
    private static final Scalar LOWER_RED = new Scalar(0, 100, 100);
    private static final Scalar UPPER_RED = new Scalar(10, 255, 255);

    private static final int RECT_SIZE = 200;

    // Define the areas for detection
    private static final Rect LEFT_RECT = new Rect(new Point(10, 10), new Point(150 + RECT_SIZE, 100 + RECT_SIZE));
    private static final Rect CENTER_RECT = new Rect(new Point(450, 10), new Point(250 + RECT_SIZE, 100 + RECT_SIZE));
    private static final Rect RIGHT_RECT = new Rect(new Point(850, 10), new Point(450 + RECT_SIZE, 100 + RECT_SIZE));

    private Mat hsvMat = new Mat();
    private Mat redMask = new Mat();

    private boolean isRedLeftDetected = false;
    private boolean isRedCenterDetected = false;
    private boolean isRedRightDetected = false;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the image from RGB to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold the HSV image to get only red colors
        Core.inRange(hsvMat, LOWER_RED, UPPER_RED, redMask);

        // Find contours of the masked regions
        List<MatOfPoint> redContours = new ArrayList<>();
        Imgproc.findContours(redMask, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Reset detection flags
        isRedLeftDetected = false;
        isRedCenterDetected = false;
        isRedRightDetected = false;

        // Check if any red contours are within the respective rectangles
        for (MatOfPoint contour : redContours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            if (LEFT_RECT.contains(new Point(boundingRect.x, boundingRect.y))) {
                isRedLeftDetected = true;
            }
            if (CENTER_RECT.contains(new Point(boundingRect.x, boundingRect.y))) {
                isRedCenterDetected = true;
            }
            if (RIGHT_RECT.contains(new Point(boundingRect.x, boundingRect.y))) {
                isRedRightDetected = true;
            }
        }

        // Draw rectangles around the detection areas
        Imgproc.rectangle(input, LEFT_RECT, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(input, CENTER_RECT, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(input, RIGHT_RECT, new Scalar(255, 0, 0), 2);

        return input;
    }

    public boolean isRedLeftDetected() {
        return isRedLeftDetected;
    }

    public boolean isRedCenterDetected() {
        return isRedCenterDetected;
    }

    public boolean isRedRightDetected() {
        return isRedRightDetected;
    }
}