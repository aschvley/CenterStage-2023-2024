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

public class ColorDetectionPipelineBlue extends OpenCvPipeline {

    // Define the colors in HSV space for blue
    private static final Scalar LOWER_BLUE = new Scalar(100, 150, 70);
    private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);

    private static final int RECT_SIZE = 200;

    // Define the areas for detection
    private static final Rect LEFT_RECT = new Rect(new Point(10, 10), new Point(150 + RECT_SIZE, 100 + RECT_SIZE));
    private static final Rect CENTER_RECT = new Rect(new Point(450, 10), new Point(250 + RECT_SIZE, 100 + RECT_SIZE));
    private static final Rect RIGHT_RECT = new Rect(new Point(850, 10), new Point(450 + RECT_SIZE, 100 + RECT_SIZE));

    private Mat hsvMat = new Mat();
    private Mat blueMask = new Mat();

    private boolean isBlueLeftDetected = false;
    private boolean isBlueCenterDetected = false;
    private boolean isBlueRightDetected = false;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the image from RGB to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold the HSV image to get only blue colors
        Core.inRange(hsvMat, LOWER_BLUE, UPPER_BLUE, blueMask);

        // Find contours of the masked regions
        List<MatOfPoint> blueContours = new ArrayList<>();
        Imgproc.findContours(blueMask, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Reset detection flags
        isBlueLeftDetected = false;
        isBlueCenterDetected = false;
        isBlueRightDetected = false;

        // Check if any blue contours are within the respective rectangles
        for (MatOfPoint contour : blueContours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            if (LEFT_RECT.contains(new Point(boundingRect.x, boundingRect.y))) {
                isBlueLeftDetected = true;
            }
            if (CENTER_RECT.contains(new Point(boundingRect.x, boundingRect.y))) {
                isBlueCenterDetected = true;
            }
            if (RIGHT_RECT.contains(new Point(boundingRect.x, boundingRect.y))) {
                isBlueRightDetected = true;
            }
        }

        // Draw rectangles around the detection areas
        Imgproc.rectangle(input, LEFT_RECT, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(input, CENTER_RECT, new Scalar(255, 0, 0), 2);
        Imgproc.rectangle(input, RIGHT_RECT, new Scalar(255, 0, 0), 2);

        return input;
    }

    public boolean isBlueLeftDetected() {
        return isBlueLeftDetected;
    }

    public boolean isBlueCenterDetected() {
        return isBlueCenterDetected;
    }

    public boolean isBlueRightDetected() {
        return isBlueRightDetected;
    }
}