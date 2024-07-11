package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "Rojo Pizarra")
public class RojoPizarra extends LinearOpMode {

    private DcMotor izqTrasero;
    private DcMotor izqFrente;
    private DcMotor derTrasero;
    private DcMotor derFrente;
    OpenCvCamera webcam;
    ColorDetectionRed colorDetectionPipeline;

    @Override
    public void runOpMode() {
        initializeMotors();
        initializeCamera();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        boolean actionExecuted = false;
        double power = 0.7;

        while (opModeIsActive() && !actionExecuted) {
            int redLeftCount = colorDetectionPipeline.getRedLeftCount();
            int redCenterCount = colorDetectionPipeline.getRedCenterCount();
            int redRightCount = colorDetectionPipeline.getRedRightCount();

            if (redLeftCount > redCenterCount && redLeftCount > redRightCount) {
                telemetry.addData("Color", "Rojo en Izquierda");

                moverAdelante( 900, 0.5);
                resetEncoders();
                rotarIzquierda(560, 1); //pone pixel en prop
                resetEncoders();
                moverAtras(340,0.5);
                resetEncoders();
                moverDerecha(470, power);
                resetEncoders();
                moverAtras(940, power); //se estaciona
                resetEncoders(); //hasta aca llega
                actionExecuted = true; // Se ha ejecutado una acción

            } else if (redCenterCount > redLeftCount && redCenterCount > redRightCount) {
                telemetry.addData("Color", "Rojo en Centro");

                resetEncoders();
                moverAdelante(900, 0.5); //pone pixel en prop
                resetEncoders();
                moverAtras(500, 0.5);
                resetEncoders();
                rotarDerecha(560, 1);
                resetEncoders();
                moverAdelante(980, 0.5);
                resetEncoders();
                moverDerecha(300, 0.4); //se estaciona
                resetEncoders();//hasta aca llega
                actionExecuted = true; // Se ha ejecutado una acción

            } else if (redRightCount > redLeftCount && redRightCount > redCenterCount) {
                telemetry.addData("Color", "Rojo en Derecha");

                moverAdelante(900, 0.5);
                resetEncoders();
                rotarDerecha(560, 1); //pone pixel en prop
                resetEncoders();
                moverAtras(340, power);
                resetEncoders();
                moverIzquierda(750, power);
                resetEncoders();
                moverAdelante(940, power); //se estaciona
                resetEncoders();//ya hasta aca llega
                actionExecuted = true; // Se ha ejecutado una acción
            } else {
                telemetry.addData("Color", "No detectado");
            }
            telemetry.update();
            sleep(100);
        }
    }

    private void initializeMotors() {
        izqTrasero = hardwareMap.get(DcMotor.class, "IzqTrasero");
        izqFrente = hardwareMap.get(DcMotor.class, "IzqFrente");
        derTrasero = hardwareMap.get(DcMotor.class, "DerTrasero");
        derFrente = hardwareMap.get(DcMotor.class, "DerFrente");

        izqTrasero.setDirection(DcMotor.Direction.REVERSE);
        izqFrente.setDirection(DcMotor.Direction.REVERSE);
        derTrasero.setDirection(DcMotor.Direction.FORWARD);
        derFrente.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();
        setRunUsingEncoders();
    }

    private void resetEncoders() {
        izqTrasero.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        izqFrente.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        derTrasero.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        derFrente.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunUsingEncoders() {
        izqTrasero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        izqFrente.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        derTrasero.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        derFrente.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void setRunToPosition() {
        izqTrasero.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        izqFrente.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        derTrasero.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        derFrente.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    private void setTargetPosition(int lf, int rf, int lr, int rr) {
        izqFrente.setTargetPosition(lf);
        derFrente.setTargetPosition(rf);
        izqTrasero.setTargetPosition(lr);
        derTrasero.setTargetPosition(rr);
        setRunToPosition();
    }

    private void setMotorPower(double lf, double rf, double lr, double rr) {
        izqFrente.setPower(lf);
        derFrente.setPower(rf);
        izqTrasero.setPower(lr);
        derTrasero.setPower(rr);
    }

    private void waitForMotors() {
        while (opModeIsActive() &&
                (izqFrente.isBusy() || derFrente.isBusy() || izqTrasero.isBusy() || derTrasero.isBusy())) {
            telemetry.addData("Motor Status", "Waiting for motors to finish");
            telemetry.update();
            idle();
        }
    }

    private void moverAdelante(int distance, double power) {
        setTargetPosition(distance, distance, distance, distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    private void moverAtras(int distance, double power) {
        setTargetPosition(-distance, -distance, -distance, -distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    private void moverIzquierda(int distance, double power) {
        setTargetPosition(-distance, distance, distance, -distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    private void moverDerecha(int distance, double power) {
        setTargetPosition(distance, -distance, -distance, distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    private void rotarIzquierda(int distance, double power) {
        setTargetPosition(-distance, distance, -distance, distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    private void rotarDerecha(int distance, double power) {
        setTargetPosition(distance, -distance, distance, -distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    private void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        colorDetectionPipeline = new ColorDetectionRed();
        webcam.setPipeline(colorDetectionPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "Camera could not be opened!");
                telemetry.update();
            }
        });
    }

    public static class ColorDetectionRed extends OpenCvPipeline {
        private static final Scalar LOWER_RED1 = new Scalar(0, 100, 100);
        private static final Scalar UPPER_RED1 = new Scalar(10, 255, 255);
        private static final Scalar LOWER_RED2 = new Scalar(160, 100, 100);
        private static final Scalar UPPER_RED2 = new Scalar(179, 255, 255);

        private static final Rect LEFT_RECT = new Rect(new Point(5, 350), new Point(395, 720));
        private static final Rect CENTER_RECT = new Rect(new Point(445, 350), new Point(835, 720));
        private static final Rect RIGHT_RECT = new Rect(new Point(885, 350), new Point(1275, 720));

        private Mat hsvMat = new Mat();
        private Mat redMask = new Mat();

        private int redLeftCount = 0;
        private int redCenterCount = 0;
        private int redRightCount = 0;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsvMat, LOWER_RED1, UPPER_RED1, redMask);
            Mat redMask2 = new Mat();
            Core.inRange(hsvMat, LOWER_RED2, UPPER_RED2, redMask2);
            Core.addWeighted(redMask, 1.0, redMask2, 1.0, 0.0, redMask);
            redMask2.release();

            // Validate rectangle coordinates
            Rect validLeftRect = validateRect(LEFT_RECT, input);
            Rect validCenterRect = validateRect(CENTER_RECT, input);
            Rect validRightRect = validateRect(RIGHT_RECT, input);

            redLeftCount = Core.countNonZero(new Mat(redMask, validLeftRect));
            redCenterCount = Core.countNonZero(new Mat(redMask, validCenterRect));
            redRightCount = Core.countNonZero(new Mat(redMask, validRightRect));

            Imgproc.rectangle(input, validLeftRect, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, validCenterRect, new Scalar(255, 0, 0), 2);
            Imgproc.rectangle(input, validRightRect, new Scalar(255, 0, 0), 2);

            return input;
        }

        private Rect validateRect(Rect rect, Mat mat) {
            int x = Math.max(0, rect.x);
            int y = Math.max(0, rect.y);
            int width = Math.min(rect.width, mat.cols() - x);
            int height = Math.min(rect.height, mat.rows() - y);
            return new Rect(x, y, width, height);
        }

        public int getRedLeftCount() {
            return redLeftCount;
        }

        public int getRedCenterCount() {
            return redCenterCount;
        }

        public int getRedRightCount() {
            return redRightCount;
        }
    }
}
