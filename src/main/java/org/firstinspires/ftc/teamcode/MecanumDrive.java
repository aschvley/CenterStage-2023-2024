package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive {
    private DcMotor izqTrasero;
    private DcMotor izqFrente;
    private DcMotor derTrasero;
    private DcMotor derFrente;
    private LinearOpMode opMode;

    public MecanumDrive(LinearOpMode opMode) {
        this.opMode = opMode;
        initializeMotors();
    }

    private void initializeMotors() {
        izqTrasero = opMode.hardwareMap.get(DcMotor.class, "IzqTrasero");
        izqFrente = opMode.hardwareMap.get(DcMotor.class, "IzqFrente");
        derTrasero = opMode.hardwareMap.get(DcMotor.class, "DerTrasero");
        derFrente = opMode.hardwareMap.get(DcMotor.class, "DerFrente");

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

    public void setTargetPosition(int lf, int rf, int lr, int rr) {
        izqFrente.setTargetPosition(lf);
        derFrente.setTargetPosition(rf);
        izqTrasero.setTargetPosition(lr);
        derTrasero.setTargetPosition(rr);
        setRunToPosition();
    }

    public void setMotorPower(double lf, double rf, double lr, double rr) {
        izqFrente.setPower(lf);
        derFrente.setPower(rf);
        izqTrasero.setPower(lr);
        derTrasero.setPower(rr);
    }

    private void waitForMotors() {
        while (opMode.opModeIsActive() &&
                (izqFrente.isBusy() || derFrente.isBusy() || izqTrasero.isBusy() || derTrasero.isBusy())) {
            opMode.idle();
        }
    }

    public void avanzar(int distance, double power) {
        setTargetPosition(distance, distance, distance, distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    public void retroceder(int distance, double power) {
        setTargetPosition(-distance, -distance, -distance, -distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    public void izquierda(int distance, double power) {
        setTargetPosition(-distance, distance, distance, -distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    public void derecha(int distance, double power) {
        setTargetPosition(distance, -distance, -distance, distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    public void rotarIzquierda(int distance, double power) {
        setTargetPosition(-distance, distance, -distance, distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }

    public void rotarDerecha(int distance, double power) {
        setTargetPosition(distance, -distance, distance, -distance);
        setMotorPower(power, power, power, power);
        waitForMotors();
    }
}
