package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.interfaces.Vision;

@Autonomous(name = "Base")
public class BaseAutonomous extends BaseOpMode {
    protected double inchValue = 6.3;
    protected boolean LOOP_AUTONOMOUS = false;
    public void Init() {

    }

    public void Run() {
        telemetry.addLine("This is the Base Autonomous mode.");
        telemetry.addLine("Please run an OpMode in the *-Autonomous/ categories.");
        telemetry.update();
    }

    public void setAutonomousAsLooping() {
        LOOP_AUTONOMOUS = false;
    }

    @Override
    public void runOpMode() {

        Init();

        driveInit();
        shooterInit();
        shooterLedInit();
        imuInit();
        // aimInit();
        gooberInit();
        cameraInit();
        //colorSensorInit();
        telemetryInit();
        runtime.reset();

        autoTelemetryLoop();

        if (gamepad1.aWasPressed()) {
            inchValue += 4;
        }
        else if (gamepad1.bWasPressed()) {
            if (!(inchValue == 0)) {
                inchValue -= 4;
            }
        }

        waitForStart();

        do {
            Run();
        }
        while (LOOP_AUTONOMOUS && opModeIsActive());

        sleep(1000);
    }

    @Override
    public void autoDriveInitOverride() {
        drive.resetEncoders();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting hat",  "%7d :%7d",
                drive.getCurrentPosition()[1],
                drive.getCurrentPosition()[3]);
        telemetry.update();
    }

    public void drive(double speed,
                      double leftInches, double rightInches,
                      double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = drive.getCurrentPosition()[1] + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = drive.getCurrentPosition()[3] + (int)(rightInches * COUNTS_PER_INCH);

            drive.setTargetPosition(newLeftTarget,newRightTarget);
            // Turn On RUN_TO_POSITION
            drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            drive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    drive.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        drive.getCurrentPosition()[1], drive.getCurrentPosition()[3]);
                telemetry.update();
            }

            // Stop all motion;
            drive.stop();

            // Turn off RUN_TO_POSITION
            drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void driveCrabe(double speed,
                           double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = drive.getCurrentPosition()[1] + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = drive.getCurrentPosition()[3] + (int)(rightInches * COUNTS_PER_INCH);

            drive.setTargetPositionCrabe(newLeftTarget,newRightTarget);

            // Turn On RUN_TO_POSITION
            drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.
            runtime.reset();
            drive.setPower(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    drive.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        drive.getCurrentPosition()[1],drive.getCurrentPosition()[3]);
                telemetry.update();
            }

            // Stop all motion;
            drive.stop();

            // Turn off RUN_TO_POSITION
            drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    @Override
    public void autoTelemetryLoop() {
        telemetry.addData("status", "runtime: " + runtime);
        telemetry.addData("inches", inchValue);

        telemetry.update();
    }

    public void intake(long time_ms) {
        goober.modPower(-1);
        sleep(time_ms);
        goober.modPower(0);
    }

    public void alignFieldGoal(double timeout_ms, double target_distance, Vision.UpdateGoalAprilTagGoal goalDetect, double yaw_offset) {
        boolean aligned = false;
        ElapsedTime alignRunTime = new ElapsedTime();
        alignRunTime.reset();

        while (
            !aligned &&
            alignRunTime.milliseconds() < timeout_ms &&
            opModeIsActive()
        ) {
            aligned = alignAprilTag(target_distance, goalDetect, yaw_offset);
        }

        telemetry.addData("Align", aligned ? "SUCCESS" : "TIMEOUT");
        telemetry.addData("took", alignRunTime.seconds());
        telemetry.update();
    }

    public void rotate(double timeout_ms, double target_rotation) {
        boolean turned = false;
        ElapsedTime turnRunTime = new ElapsedTime();
        turnRunTime.reset();

        while (
            !turned &&
            turnRunTime.milliseconds() < timeout_ms &&
            opModeIsActive()
        ) {
            turned = rotateRobot(target_rotation);
        }

        telemetry.addData("Turn", turned ? "SUCCESS" : "TIMEOUT");
        telemetry.addLine("took %7d s", turnRunTime.seconds());
        telemetry.update();
    }
}
