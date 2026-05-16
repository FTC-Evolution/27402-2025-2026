package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Drive {

    protected final DcMotor frontLeftDrive;
    protected final DcMotor backLeftDrive;
    protected final DcMotor frontRightDrive;
    protected final DcMotor backRightDrive;

    public Drive(DcMotor fld, DcMotor bld, DcMotor frd, DcMotor brd) {
        frontLeftDrive = fld;
        backLeftDrive = bld;
        frontRightDrive = frd;
        backRightDrive = brd;

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetEncoders() {
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int[] getCurrentPosition() {
        return new int[] {
            frontLeftDrive.getCurrentPosition(),
            backLeftDrive.getCurrentPosition(),
            frontRightDrive.getCurrentPosition(),
            backRightDrive.getCurrentPosition(),
        };
    }

    public void setTargetPosition(int newLeftTarget, int newRightTarget) {
        backLeftDrive.setTargetPosition(newLeftTarget);
        frontLeftDrive.setTargetPosition(newLeftTarget);

        backRightDrive.setTargetPosition(newRightTarget);
        frontRightDrive.setTargetPosition(newRightTarget);
    }

    public void setTargetPositionCrabe(int newLeftTarget, int newRightTarget) {
        backRightDrive.setTargetPosition(newLeftTarget);
        frontLeftDrive.setTargetPosition(newLeftTarget);

        backLeftDrive.setTargetPosition(newRightTarget);
        frontRightDrive.setTargetPosition(newRightTarget);
    }

    public void setMode(DcMotor.RunMode mode) {
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
    }

    public void setPower(double power) {
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
    }

    public void setPowerGranular(
        double frontLeftPower,
        double backLeftPower,
        double frontRightPower,
        double backRightPower
    ) {
        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    public boolean isBusy() {
        return (
            backLeftDrive.isBusy() ||
            backRightDrive.isBusy() ||
            frontLeftDrive.isBusy() ||
            frontRightDrive.isBusy()
        );
    }

    public void stop() {
        setPower(0);
    }
}
