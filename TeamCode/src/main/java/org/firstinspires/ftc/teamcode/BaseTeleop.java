package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.interfaces.Brain;
import org.firstinspires.ftc.teamcode.interfaces.Shooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="Teleop 27402 Sec 2")
public class BaseTeleop extends BaseOpMode {
    double counter1 = 0;
    double counter2 = 0;
    boolean end = false;

    double DRIVE_STICK_DEADZONE = 0.1;

    String winner = "none";

    @Override
    public void runOpMode() {


        driveInit();
        shooterInit();
        gooberInit();
        // colorSensorInit();
        telemetryInit();
        imuInit();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // buttonLoop();
            driveLoop();
            shooterLoop();
            gooberLoop();
            // colorSensorLoop();
            telemetryLoop();
        }

    }

    @Override
    public void aprilTagLoop() {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.metadata != null) {
                if (detection.metadata.name.contains("Obelisk")) {
                    obelisk = obeliskPositions.get(detection.id);
                } else {
                    fieldSide = fieldSidePositions.get(detection.id);

                    if (gamepad1.leftBumperWasPressed()) {
                        if (Math.abs(detection.ftcPose.x) > APRILTAG_TOLERANCE_SIDE) ;
                        {
                            autoDriveCrabe(DRIVE_SPEED, detection.ftcPose.x, -detection.ftcPose.x, 5.0);
                        }
                    }

                }


            }

        }
    }

    public void driveLoop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;



        // i add cool deadzones WOW
        if (Math.abs(y) < DRIVE_STICK_DEADZONE) {
            y = 0;
        }

        if (Math.abs(x) < DRIVE_STICK_DEADZONE) {
            x = 0;
        }

        if (Math.abs(rx) < DRIVE_STICK_DEADZONE) {
            rx = 0;
        }

        if (gamepad1.startWasPressed()) {
            brain.resetYaw();
        }

        double botDirection = brain.getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botDirection) - y * Math.sin(-botDirection);
        double rotY = x * Math.sin(-botDirection) + y * Math.cos(-botDirection);

        rotX *= 1.1;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
    }

    public void fieldOrientationLoop() {
        if (gamepad1.xWasPressed()) {
            brain.resetYaw();
        }
    }

    public void autoDriveSetup() {
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void autoDriveCancel() {
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void autoDriveCrabe(double speed,
                               double leftInches, double rightInches, double timeoutS) {
        autoDriveSetup();
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = backLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = backRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            backRightDrive.setTargetPosition(-newLeftTarget);
            frontLeftDrive.setTargetPosition(newLeftTarget);

            backLeftDrive.setTargetPosition(newRightTarget);
            frontRightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backLeftDrive.isBusy() && backRightDrive.isBusy() && frontRightDrive.isBusy() && frontLeftDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);


            // Turn off RUN_TO_POSITION
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            sleep(250);   // optional pause after each move.
        }
    }

    public void buttonLoop() {


        // telemetry.addLine("get 50 to win");

        if (end) {
            return;
        }
        if (gamepad1.xWasPressed()) {
            counter1 += 1;
        }

        if (gamepad2.xWasPressed()) {
            counter2 += 1;
        } else if (counter1 >= 50) {
            winner = "player 1";
            end = true;

        } else if (counter2 >= 50) {
            winner = "player 2";
            end = true;
        }

        telemetry.addData("player 1 score", counter1);
        telemetry.addData("player 2 score", counter2);
        telemetry.addData("winner", winner);
        telemetry.addData("game over", end);

        telemetry.update();
    }


}