package org.firstinspires.ftc.teamcode.autonomous;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

public class BaseAutonomous extends LinearOpMode {
    boolean showImuTelemetry = false;
    boolean showShooterTelemetry = false;
    boolean showColorTelemetry = false;
    boolean showAimingTelemetry = false;
    boolean showGooberTelemetry = false;
    boolean showCameraTelemetry = false;


    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotor goober;
    private DcMotor goober2;

    private NormalizedColorSensor sensor;
    private Servo shooteraim;
    final double SHOOTER_TICKS_PER_REV = 28;

    public final ElapsedTime runtime = new ElapsedTime();

    private String obelisk;
    private String fieldSide;

    Map<Integer, String> obeliskPositions;
    Map<Integer, String> fieldSidePositions;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;


    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;
    AngleUnit angle = AngleUnit.DEGREES;
    IMU.Parameters imup = new IMU.Parameters(orientationOnRobot);

    private double shooterPower = 40; // Perfect speed to throw the ball when at the point of the launch triangle on the field
    double shooterTPS = shooterPower * SHOOTER_TICKS_PER_REV;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_DISTANCE_INCHES   = 12.25;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     INCHES_PER_DEGREE       = (WHEEL_DISTANCE_INCHES * Math.PI) / 360;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double     SHOOTER_P              = 90;
    static final double     SHOOTER_I              = 1.95;
    static final double     SHOOTER_D              = 0.1;
    static final double     SHOOTER_F              = 0;
    static final String[] possiblePaths = {"default", "avancer", "reculer","practice","crabeLEFT", "crabeRIGHT", "tournerCLOCK", "tournerCOUNTER","one","short","testCombo","richel"};
    int currentPath = 0;
    double inchValue = 6.3;

    public double degreesToInches(double degrees) {
        return (degrees / 360) * INCHES_PER_DEGREE;
    }

    public void runAutonomous() {
        telemetry.addLine("Do not run this opMode!");
        telemetry.update();
    }

    public void runOpMode() {


        InitDrive();
        shooterInit();
        // aimInit();
        // buttoninnit();
        // gooberInit();
        initAprilTag();
        //colorSensorInit();
        telemetryInit();
        runtime.reset();

        earlyTelemetryLoop();

        if (gamepad1.yWasPressed()) {
            if (currentPath < possiblePaths.length - 1) {
                currentPath += 1;
            } else {
                currentPath = 0;
            }}
        if (gamepad1.aWasPressed()) {
            inchValue += 4;
        }
        else if (gamepad1.bWasPressed()) {
            if (!(inchValue == 0)) {
                inchValue -= 4;
            }
        }

        waitForStart();

        runAutonomous();

        while (opModeIsActive()) {

            telemetryLoop();
        }

        // buttonLoop();
        // boucleDrive();
        // aimLoop();
        // shooterLoop();
        // gooberLoop();
        //aprilTagLoop();
        //colorSensorLoop();


    }

    public void initAprilTag() {

        showCameraTelemetry = true;

        obeliskPositions = new HashMap<Integer, String>();
        obeliskPositions.put(21,"GPP");
        obeliskPositions.put(22,"PGP");
        obeliskPositions.put(23,"PPG");


        fieldSidePositions = new HashMap<Integer, String>();
        fieldSidePositions.put(20,"blue");
        fieldSidePositions.put(24,"red");

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        // if (USE_WEBCAM) {
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

    }

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
                        turnBoucleDrive(TURN_SPEED, fieldSideYaw, -fieldSideYaw, 5.0);
                    }


                }

            }

        }
    }
    public void shooterInit() {
        showShooterTelemetry = true;
        shooter1 = hardwareMap.get(DcMotorEx.class, "leftshooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "rightshooter");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        shooter2.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        // Reset encoders
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void gooberInit(){
        showGooberTelemetry = true;
        goober = hardwareMap.get(DcMotor.class, "goober");

    }

    public void gooberLoop(){

        // Change to Gamepad 1, Gamepad 2 is for testing purposes
        // Goober motor variable is goober\



    }
    public void shooterLoop() {
        shooterTPS = shooterPower * SHOOTER_TICKS_PER_REV;

        shooter1.setVelocity(-gamepad2.right_trigger * shooterTPS);
        shooter2.setVelocity(gamepad2.right_trigger * shooterTPS);

        if (gamepad2.aWasPressed()) {
            if (shooterPower <= 0){ return; }
            shooterPower -= 5;
        }
        /*if (gamepad2.yWasPressed()) {
            if (shooterPower >= 100){ return; }
            if (shooterPower == 95) {
                shooterPower += 5;
                return;
            }
            shooterPower += 10;
        }*/
        if (gamepad2.yWasPressed()) {
            if (shooterPower >= 100){ return; }
            shooterPower += 5;
        }
        /*if (gamepad2.aWasPressed()) {
            if (shooterPower <= 0){ return; }
            if (shooterPower <= 5) {
                shooterPower -= 5;
            }
            shooterPower -= 10;
        }*/

        if (gamepad2.bWasPressed()) {
            shooterPower = 40;
        }
        if (gamepad2.xWasPressed()) {
            shooterPower = 45;
        }
    }





    public void InitDrive() {
        showImuTelemetry = true;
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftDrive = hardwareMap.dcMotor.get("fld");
        backLeftDrive = hardwareMap.dcMotor.get("bld");
        frontRightDrive = hardwareMap.dcMotor.get("frd");
        backRightDrive = hardwareMap.dcMotor.get("brd");



        // Initialize the drive system variables.

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting hat",  "%7d :%7d :%7d :%7d",
                backLeftDrive.getCurrentPosition(),
                backRightDrive.getCurrentPosition());
        telemetry.update();
    }

    public void turnBoucleDrive(double speed,
                                double leftDegrees, double rightDegrees,
                                double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        int leftDegreeAngle;
        int rightDegreeAngle;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftDegrees * INCHES_PER_DEGREE * COUNTS_PER_INCH);
            newRightTarget = backRightDrive.getCurrentPosition() + (int)(rightDegrees * INCHES_PER_DEGREE * COUNTS_PER_INCH);

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
    public void boucleDrive(double speed,
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

    public void boucleDriveCrabe(double speed,
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

    public void boucleShoot(double speed, double timeoutS){
        shooterTPS = speed * SHOOTER_TICKS_PER_REV;
        shooter1.setVelocity(shooterTPS);
        shooter2.setVelocity(shooterTPS);
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS)) {
            // Display it for the drivers
            telemetry.addData("Shooter desired turns per second", shooterPower);
            telemetry.addData("Shooters desired velocity",  shooterTPS);
            telemetry.addData("Shooter1 velocity", shooter1.getVelocity());
            telemetry.addData("Shooter2 velocity" ,shooter2.getVelocity());
            telemetry.update();
        }

        // Stop all motion;
        shooter1.setPower(0);
        shooter2.setPower(0);

        sleep(250);   // optional pause after each move.
    }

    public void runPath(String path) {
        switch (path) {
            case "testCombo":
                boucleDrive(DRIVE_SPEED,  48,  48, 5.0);
                boucleDrive(TURN_SPEED,   12, -12, 4.0);
                boucleDrive(DRIVE_SPEED, -24, -24, 4.0);
                break;
            case "one":
                boucleDrive(DRIVE_SPEED,  65.80,  65.8, 3.0);
                boucleDrive(TURN_SPEED,   24, -24, 4.0);
                boucleDrive(DRIVE_SPEED, -24, -24, 4.0);
                break;
            case "short":
                boucleDrive(DRIVE_SPEED, 54, 54, 0.0);  // S1: Forward 47 Inches with 5 Sec timeout
                boucleDrive(TURN_SPEED, 20, -20, 0.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                boucleDrive(DRIVE_SPEED, 30, 30, 0.0);  // S3: Reverse 24 Inches with 4 Sec timeout
                boucleDrive(TURN_SPEED, 8, -8, 0.0);  // S3: Reverse 24 Inches with 4 Sec timeout
                boucleDrive(DRIVE_SPEED, 19, 19, 1.5);  // S3: Reverse 24 Inches with 4 Sec timeout
                boucleDrive(TURN_SPEED, 6, -6, 0.0);  // S3: Reverse 24 Inches with 4 Sec timeout
                boucleDrive(DRIVE_SPEED, 44, 44, 0.0);  // S3: Reverse 24 Inches with 4 Sec timeout
                boucleDrive(TURN_SPEED, 10, -10, 2.5);  // S3: Reverse 24 Inches with 4 Sec timeout
                boucleDrive(DRIVE_SPEED, 70, 70, 0.0);  // S3: Reverse 24 Inches with 4 Sec timeout
                boucleDrive(TURN_SPEED, 7, -7, 6.7);  // S3: Reverse 24 Inches with 4 Sec timeout
                break;
            case "avancer":
                boucleDrive(DRIVE_SPEED,  inchValue,  inchValue, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                break;
            case "reculer":
                boucleDrive(DRIVE_SPEED,  -inchValue,  -inchValue, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                break;
            case "tournerCLOCK":
                boucleDrive(DRIVE_SPEED,  inchValue,  -inchValue, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                break;
            case "tournerCOUNTER":
                boucleDrive(DRIVE_SPEED,  -inchValue,  inchValue, 5.0);  // S1:
                break;
            case "crabeLEFT":
                boucleDriveCrabe(DRIVE_SPEED, inchValue, -inchValue, 5.0);
                break;
            case "crabeRIGHT":
                boucleDriveCrabe(DRIVE_SPEED, -inchValue, inchValue, 5.0);
                break;
            case "richel":
                boucleDrive(DRIVE_SPEED, 77.5, 77.5, 0.5);
                boucleDrive(TURN_SPEED, 12, -12, 0.5);
                break;
            case "practice":
                boucleDrive(DRIVE_SPEED, -6.3, -6.3, 0.5);
                break;
            case "default":
                boucleDriveCrabe(DRIVE_SPEED, -72, 72, 5.0);
                boucleDrive(TURN_SPEED, degreesToInches(45), -degreesToInches(45), 5.0);
                break;
        }
        telemetry.addData("Path", "Complete " + path);
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    public void colorSensorInit() {
        showColorTelemetry = true;
        sensor = hardwareMap.get(NormalizedColorSensor.class,"colorsensor");
    }
    public void colorSensorLoop(){
        // La variable du senseur de couleur est sensor
        // Pour l'invoquer en tant que senseur de distance, on doit mettre des parentheses, comme ceci: (OpticalDistanceSensor sensor).
    }
    public void telemetryInit(){
        telemetry.addData("status", "initialized");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imup);
        imu.resetYaw();
    }

    @SuppressLint("DefaultLocale")
    public void telemetryLoop(){

        telemetry.addData("status", "runtime: " + runtime);
        telemetry.addData("current path id", currentPath);
        telemetry.addData("current path", possiblePaths[currentPath]);
        telemetry.addData("inches", inchValue);

        if (showImuTelemetry) {
            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(angle));
            telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(angle));
            telemetry.addData("roll", imu.getRobotYawPitchRollAngles().getRoll(angle));

            telemetry.addData("angular velocity x", imu.getRobotAngularVelocity(angle).xRotationRate);
            telemetry.addData("angular velocity y", imu.getRobotAngularVelocity(angle).yRotationRate);
            telemetry.addData("angular velocity z", imu.getRobotAngularVelocity(angle).zRotationRate);
        }

        if (showCameraTelemetry) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("number of apriltags", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (%d) %s", detection.id, detection.metadata.name));telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            telemetry.addLine("key:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            telemetry.addData("Last seen Pattern", obelisk);
            telemetry.addData("Last seen Target", fieldSide);
        }

        if (showShooterTelemetry) {
            telemetry.addData("left Shooter power", shooter1.getPower());
            telemetry.addData("right Shooter power", shooter2.getPower());
        }

        if (showColorTelemetry) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            int colorCode = colors.toColor();
            String ballColor = "";

            telemetry.addData("Hue", JavaUtil.colorToHue(colorCode));
            telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colorCode));
            telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colorCode));
            telemetry.addData("Color", JavaUtil.colorToText(colorCode));

            telemetry.addData("Alpha", "%.3f", colors.alpha);

            telemetry.addData("Light level", "%.3f", ((OpticalDistanceSensor) sensor).getLightDetected());

            if ((JavaUtil.colorToHue(colorCode) >= 200) && (JavaUtil.colorToHue(colorCode) <= 300)) {
                ballColor = "purple";
            }
            else if ((JavaUtil.colorToHue(colorCode) >= 30) && (JavaUtil.colorToHue(colorCode) <= 199)) {
                ballColor = "green";
            } else {
                ballColor = "unknown";
            }

            telemetry.addData("Ball color", ballColor);
        }

        if (showGooberTelemetry) {
            telemetry.addData("Goober power", goober.getPower());

        }

        telemetry.addLine("Press Y to switch modes");
        telemetry.addLine("Press Play to start modes");

        telemetry.update();
    }

    public void earlyTelemetryLoop() {
        telemetry.addData("status", "runtime: " + runtime);
        telemetry.addData("current path id", currentPath);
        telemetry.addData("current path", possiblePaths[currentPath]);
        telemetry.addData("inches", inchValue);

        telemetry.update();
    }
}
