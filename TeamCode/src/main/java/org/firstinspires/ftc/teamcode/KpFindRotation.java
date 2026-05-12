package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.SimplePID;

@TeleOp(name="KP Finder - Rotation")
@Disabled
public class KpFindRotation extends BaseOpMode {
    double ROTATION_KP = 0;
    double ROTATION_I = 0;
    double ROTATION_KD = 0;
    double TARGET_ROTATION = 135;

    @Override
    public void runOpMode() {


        driveInit();
        shooterInit();
        gooberInit();
        cameraInit();
        shooterLedInit();
        telemetryInit();
        imuInit();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                rotate(2500, TARGET_ROTATION);
            }

            shooterLoop();
            gooberLoop();
            kpFindLoop();
            telemetryLoop();
        }

    }

    public void updatePID() {
        robotOrientationPID = new SimplePID(
                ROTATION_KP,
                ROTATION_I,
                ROTATION_KD,
                -0.3,
                0.3,
                1.2
        );
    }

    public void kpFindLoop() {
        if (gamepad1.bWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                ROTATION_KP += 0.1;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                ROTATION_KP += 0.001;
                updatePID();
                return;
            }
            ROTATION_KP += 0.01;
            updatePID();
        }
        if (gamepad1.aWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                ROTATION_KP -= 0.1;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                ROTATION_KP -= 0.001;
                updatePID();
                return;
            }
            ROTATION_KP -= 0.01;
            updatePID();
        }
        if (gamepad1.yWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                ROTATION_KD += 0.05;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                ROTATION_KD += 0.0001;
                updatePID();
                return;
            }
            ROTATION_KD += 0.001;
            updatePID();
        }
        if (gamepad1.xWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                ROTATION_KD -= 0.05;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                ROTATION_KD -= 0.0001;
                updatePID();
                return;
            }
            ROTATION_KD -= 0.001;
            updatePID();
        }
        if (gamepad1.dpadUpWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                ROTATION_I += 1;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                ROTATION_I += 0.01;
                updatePID();
                return;
            }
            ROTATION_I += 0.05;
        }
        if (gamepad1.dpadDownWasPressed()) {
            if (gamepad1.left_trigger > 0.5) {
                ROTATION_I -= 1;
                updatePID();
                return;
            }
            if (gamepad1.right_trigger > 0.5) {
                ROTATION_I -= 0.01;
                updatePID();
                return;
            }
            ROTATION_I -= 0.05;
            updatePID();
        }
    }

    public void rotate(double timeout_ms, double target_rotation) {
        boolean turned = false;
        ElapsedTime turnRunTime = new ElapsedTime();
        turnRunTime.reset();

        while (!turned || turnRunTime.milliseconds() < timeout_ms) {
            turned = rotateRobot(target_rotation);
            telemetry.addData("Turned?", turned);
            telemetry.addData("runtime", turnRunTime.seconds());
            telemetry.update();
        }
    }

    public void telemetryLoop(){
        telemetry.addData("status", "runtime: " + runtime);

        telemetry.addData("yaw", brain.getYaw(AngleUnit.DEGREES));

        telemetry.addLine("Increase P until oscillations occur");
        telemetry.addLine("Increase D until no overshoot occurs");
        telemetry.addLine("If there is steady state error, increase I until corrected");

        telemetry.addData("B","P UP");
        telemetry.addData("A","P DOWN");
        telemetry.addData("Y","D UP");
        telemetry.addData("X","D DOWN");
        telemetry.addData("DpadUP","I UP");
        telemetry.addData("DpadDown","I DOWN");

        telemetry.addData("kP", ROTATION_KP);
        telemetry.addData("I", ROTATION_I);
        telemetry.addData("kD", ROTATION_KD);
        telemetry.addData("Rotation amount", TARGET_ROTATION);

        telemetry.update();
    }
}