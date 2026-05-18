package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.interfaces.Vision;

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
        cameraInit();
        shooterLedInit();
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
            cameraLoop();
            // colorSensorLoop();
            fieldOrientationLoop();
            telemetryLoop();

            led.updateConfetti(30);
        }

    }

    public void driveLoop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;



        // I add cool dead zones WOW
        if (Math.abs(y) < DRIVE_STICK_DEADZONE) {
            y = 0;
        }

        if (Math.abs(x) < DRIVE_STICK_DEADZONE) {
            x = 0;
        }

        if (Math.abs(rx) < DRIVE_STICK_DEADZONE) {
            rx = 0;
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

        drive.setPowerGranular(frontLeftPower,backLeftPower,frontRightPower,backRightPower);
    }

    public void fieldOrientationLoop() {
        if (gamepad1.xWasPressed()) {
            brain.resetYaw();
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

    public void cameraLoop() {
        vision.updateGoalAprilTag(Vision.UpdateGoalAprilTagGoal.BOTH);
        if (gamepad1.b) {
            alignAprilTag(45.0, Vision.UpdateGoalAprilTagGoal.BOTH,0);
        }

        if (gamepad1.y) {
            alignAprilTag(115.0, Vision.UpdateGoalAprilTagGoal.BOTH,34.5);
        }
    }
}