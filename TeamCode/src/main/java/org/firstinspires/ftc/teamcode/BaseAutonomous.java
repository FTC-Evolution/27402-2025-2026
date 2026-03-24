package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class BaseAutonomous extends BaseOpMode {
    AngleUnit angle = AngleUnit.DEGREES;

    static final String[] possiblePaths = {"default", "avancer", "reculer","practice","crabeLEFT", "crabeRIGHT", "tournerCLOCK", "tournerCOUNTER","one","short","testCombo","richel"};
    int currentPath = 0;
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
        // aimInit();
        gooberInit();
        imuInit();
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
    public void aprilTagLoop(){
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.metadata != null) {
                if (detection.metadata.name.contains("Obelisk")) {
                    obelisk = obeliskPositions.get(detection.id);
                } else {
                    fieldSide = fieldSidePositions.get(detection.id);

                    // [TODO] Try figuring this out with detection.ftcPose.x and the crabeBoucleDrive() function like Thierry said

                    double fieldSideYaw = -detection.ftcPose.yaw;
                    final double tolerance = 2;
                    double fieldSideDistance = detection.ftcPose.x - 70.5;
                    if (Math.abs(fieldSideYaw) > tolerance) {
                        drive(TURN_SPEED, degreesToInches(fieldSideYaw), degreesToInches(-fieldSideYaw), 5.0);
                    }


                }

            }

        }
    }

    @Override
    public void autoDriveInitOverride() {
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting hat",  "%7d :%7d",
                backLeftDrive.getCurrentPosition(),
                backRightDrive.getCurrentPosition());
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
            newLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            backLeftDrive.setTargetPosition(newLeftTarget);
            frontLeftDrive.setTargetPosition(newLeftTarget);

            backRightDrive.setTargetPosition(newRightTarget);
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
                    (backLeftDrive.isBusy() && backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
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

    public void driveCrabe(double speed,
                           double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            backRightDrive.setTargetPosition(newLeftTarget);
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
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
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
    public void modShooter(double speed) {
        shooterTPS = speed * SHOOTER_TICKS_PER_REV;
        shooter1.setVelocity(shooterTPS);
        shooter2.setVelocity(shooterTPS);

        telemetry.addData("Shooter desired turns per second", shooterPower);
        telemetry.addData("Shooters desired velocity",  shooterTPS);
        telemetry.addData("Shooter1 velocity", shooter1.getVelocity());
        telemetry.addData("Shooter2 velocity" ,shooter2.getVelocity());
        telemetry.update();
    }

    @Override
    public void autoTelemetryLoop() {
        telemetry.addData("status", "runtime: " + runtime);
        telemetry.addData("current path id", currentPath);
        telemetry.addData("current path", possiblePaths[currentPath]);
        telemetry.addData("inches", inchValue);

        telemetry.update();
    }
}
