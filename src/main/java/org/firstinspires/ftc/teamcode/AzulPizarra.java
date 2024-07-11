package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "Azul Pizarra")
public class AzulPizarra extends LinearOpMode {

    private DcMotor izqTrasero;
    private DcMotor izqFrente;
    private DcMotor derTrasero;
    private DcMotor derFrente;
    OpenCvCamera webcam;
    ColorDetectionBlue colorDetectionPipeline;

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
            int blueLeftCount = colorDetectionPipeline.getBlueLeftCount();
            int blueCenterCount = colorDetectionPipeline.getBlueCenterCount();
            int blueRightCount = colorDetectionPipeline.getBlueRightCount();

            if (blueLeftCount > blueCenterCount && blueLeftCount > blueRightCount) {
                telemetry.addData("Color", "Azul en Izquierda");
                moverAtras(900, 0.5);
                resetEncoders();
                rotarIzquierda(560, 1); //pone pixel en prop
                resetEncoders();
                moverAdelante(340,0.5);
                resetEncoders();
                moverDerecha(470, power);
                resetEncoders();
                moverAtras(940, power); //se estaciona
                resetEncoders();// ya hasta aca llega
                actionExecuted = true; // Se ha ejecutado una acción
            } else if (blueCenterCount > blueLeftCount && blueCenterCount > blueRightCount) {
                telemetry.addData("Color", "Azul en Centro");
                moverAtras(900, 0.5); //pone pixel en prop
                resetEncoders();
                moverAdelante(350,0.5);
                resetEncoders();
                rotarIzquierda(560,1);
                resetEncoders();
                moverAtras(970, power);
                resetEncoders();
                moverDerecha(200,1); //se estaciona
                resetEncoders();// ya hasta aca llega
                actionExecuted = true; // Se ha ejecutado una acción
            } else if (blueRightCount > blueLeftCount && blueRightCount > blueCenterCount) {
                telemetry.addData("Color", "Azul en Derecha");
                moverAtras(900,0.5);
                resetEncoders();
                rotarDerecha(560,1); //pone pixel en prop
                resetEncoders();
                moverAdelante(200,0.5);
                resetEncoders();
                moverIzquierda(350,0.5);
                moverAdelante(940, power); //se estaciona
                resetEncoders(); // ya hasta aca llega
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

        colorDetectionPipeline = new ColorDetectionBlue();
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

    public static class ColorDetectionBlue extends OpenCvPipeline {
        // Definir rangos de color para el azul en espacio HSV
        private static final Scalar LOWER_BLUE = new Scalar(100, 150, 50);
        private static final Scalar UPPER_BLUE = new Scalar(140, 255, 255);

        private static final int RECT_SIZE = 200;

        // Definir rectángulos para la detección
        private static final Rect LEFT_RECT = new Rect(new Point(20, 200), new Point(20 + RECT_SIZE, 200 + RECT_SIZE));
        private static final Rect CENTER_RECT = new Rect(new Point(230, 200), new Point(230 + RECT_SIZE, 200 + RECT_SIZE));
        private static final Rect RIGHT_RECT = new Rect(new Point(450, 200), new Point(450 + RECT_SIZE, 200 + RECT_SIZE));

        private Mat hsvMat = new Mat();
        private Mat blueMask = new Mat();

        private int blueLeftCount = 0;
        private int blueCenterCount = 0;
        private int blueRightCount = 0;

        @Override
        public Mat processFrame(Mat input) {
            // Convertir la imagen al espacio de color HSV
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Crear la máscara para el color azul
            Core.inRange(hsvMat, LOWER_BLUE, UPPER_BLUE, blueMask);

            // Validar coordenadas de los rectángulos
            Rect validLeftRect = validateRect(LEFT_RECT, input);
            Rect validCenterRect = validateRect(CENTER_RECT, input);
            Rect validRightRect = validateRect(RIGHT_RECT, input);

            // Contar píxeles azules en cada rectángulo
            blueLeftCount = Core.countNonZero(new Mat(blueMask, validLeftRect));
            blueCenterCount = Core.countNonZero(new Mat(blueMask, validCenterRect));
            blueRightCount = Core.countNonZero(new Mat(blueMask, validRightRect));

            // Dibujar rectángulos en la imagen de entrada
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

        public int getBlueLeftCount() {
            return blueLeftCount;
        }

        public int getBlueCenterCount() {
            return blueCenterCount;
        }

        public int getBlueRightCount() {
            return blueRightCount;
        }
    }
}