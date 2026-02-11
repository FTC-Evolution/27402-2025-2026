package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name="Mode Autonome Sec 2 27402")
public class DefaultAutonomous extends LinearOpMode {
    boolean showImuTelemetry = false;
    boolean showShooterTelemetry = false;
    boolean showColorTelemetry = false;
    boolean showAimingTelemetry = false;
    boolean showGooberTelemetry = false;
    boolean showCameraTelemetry = false;


    private DcMotor shooter1;
    private DcMotor shooter2;
    private DcMotor goober;
    private NormalizedColorSensor sensor;
    private Servo shooteraim;

    private ElapsedTime runtime = new ElapsedTime();

    private Integer divider = 1;

    double counter1 = 0;
    double counter2 = 0;

    private String obelisk;
    private String fieldSide;
    private double fieldSideDistance;
    private double fieldSideYaw;

    Map<Integer, String> obeliskPositions;
    Map<Integer, String> fieldSidePositions;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    double viseurangle = 90;
    boolean end = false;
    String winner = "none";



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

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.14159265358979323);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     WHEEL_DISTANCE_INCHES   = 12.25;//gnjdfkjhdfbdandgwgfu
    static final double     INCHES_PER_DEGREE       = (WHEEL_DISTANCE_INCHES * Math.PI) / 360;
    static final String[] possiblePaths = {"avancer", "reculer","crabeLEFT", "crabeRIGHT", "tournerCLOCK", "tournerCOUNTER","one","short","testCombo",};
    int currentPath = 0;
    int inchValue = 12;
    int degreeValue = 90;

    double leftDegreeAngle;
    double rightDegreeAngle;

    public void runOpMode() {


        InitDrive();
        // shooterInit();
        // aimInit();
        // buttoninnit();
        // gooberInit();
        initAprilTag();
        //colorSensorInit();
        telemetryInit();
        runtime.reset();

        waitForStart();



        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                if (currentPath < possiblePaths.length - 1) {
                    currentPath += 1;
                } else {
                    currentPath = 0;
                }
            }
            else if (gamepad1.xWasPressed())
            {
                runPath(possiblePaths[currentPath]);
            }
            if (gamepad1.aWasPressed()) {
                inchValue += 4;
            }
            else if (gamepad1.bWasPressed()) {
                if (!(inchValue == 0)) {
                    inchValue -= 4;
                }
            }
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
                hardwareMap.get(WebcamName.class, "webcam"), aprilTag);

    }

    public void aprilTagLoop(){
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.metadata != null) {
                if (detection.metadata.name.contains("Obelisk")) {
                    obelisk = obeliskPositions.get(detection.id);

                } else {
                    fieldSide = fieldSidePositions.get(detection.id);

                    fieldSideYaw = -detection.ftcPose.yaw;
                    turnBoucleDrive(TURN_SPEED, fieldSideYaw, -fieldSideYaw, 5.0);
                }

            }

        }
    }

    public void shooterInit() {
        showShooterTelemetry = true;
        shooter1 = hardwareMap.get(DcMotor.class, "leftshooter");
        shooter2 = hardwareMap.get(DcMotor.class, "rightshooter");
    }

    public void aimInit() {
        showAimingTelemetry = true;
        shooteraim = hardwareMap.get(Servo.class, "servo0");
        shooteraim.setPosition(viseurangle/180);
    }

    public void aimLoop() {
        if (gamepad1.dpadUpWasPressed()){
            viseurangle += 5;
        } else if (gamepad2.dpadDownWasPressed()) {
            viseurangle -= 5;
        }

        shooteraim.setPosition(viseurangle/180);
    }

    public void autoTestInit() {
        showAimingTelemetry = true;
        shooteraim = hardwareMap.get(Servo.class, "servo0");
        shooteraim.setPosition(viseurangle/180);
    }

    public void autoTestLoop() {
        if (gamepad1.dpadUpWasPressed()){
            viseurangle += 5;
        } else if (gamepad1.dpadDownWasPressed()) {
            viseurangle -= 5;
        }

        shooteraim.setPosition(viseurangle/180);
    }

    public void gooberInit(){
        showGooberTelemetry = true;
        goober = hardwareMap.get(DcMotor.class, "goober");

    }

    public void gooberLoop(){

        // Change to Gamepad 1, Gamepad 2 is for testing purposes
        // Goober motor variable is goober

    }
    public void shooterLoop() {
        // Shooter motor variables are shooter1 and shooter2
        // Change the divider variable to make the motors faster or slower
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
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        // backRightDrive.setDirection(DcMotor.Direction.REVERSE); // Reverse because of 3 gears ??????

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

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


    public void boucleDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        leftDegreeAngle = (leftInches * 180) / (Math.PI * WHEEL_DIAMETER_INCHES/2);
        rightDegreeAngle = (rightInches * 180) / (Math.PI * WHEEL_DIAMETER_INCHES/2);

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            backLeftDrive.setTargetPosition(newLeftTarget);
            frontLeftDrive.setTargetPosition(newLeftTarget);

            backRightDrive.setTargetPosition(-newRightTarget);
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

    public void turnBoucleDrive(double speed,
                            double leftDegrees, double rightDegrees,
                            double timeoutS) {
        int newLeftTarget;
        int newRightTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftDegrees * INCHES_PER_DEGREE * COUNTS_PER_INCH);
            newRightTarget = backRightDrive.getCurrentPosition() + (int)(rightDegrees * INCHES_PER_DEGREE * COUNTS_PER_INCH);

            backLeftDrive.setTargetPosition(newLeftTarget);
            frontLeftDrive.setTargetPosition(newLeftTarget);

            backRightDrive.setTargetPosition(-newRightTarget);
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


    public void runPath(String path) {
        if (Objects.equals(path, "testCombo")) {
            boucleDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            boucleDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            boucleDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        } else if (Objects.equals(path, "one")) {
            boucleDrive(DRIVE_SPEED,  65.80,  65.8, 3.0);  // S1: Forward 47 Inches with 5 Sec timeout
            boucleDrive(TURN_SPEED,   24, -24, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            boucleDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        }  else if (Objects.equals(path, "short")) {
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
        } else if (Objects.equals(path, "avancer")) {
            boucleDrive(DRIVE_SPEED,  inchValue,  inchValue, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        } else if (Objects.equals(path, "reculer")) {
            boucleDrive(DRIVE_SPEED,  -inchValue,  -inchValue, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        } else if (Objects.equals(path, "tournerCLOCK")) {
            turnBoucleDrive(TURN_SPEED, degreeValue, -degreeValue, 5.0);
        } else if (Objects.equals(path, "tournerCOUNTER")) {
            turnBoucleDrive(TURN_SPEED, -degreeValue, degreeValue, 5.0);
        } else if (Objects.equals(path,"crabeRIGHT")) {
            boucleDriveCrabe(DRIVE_SPEED, inchValue, -inchValue, 5.0);
        } else if (Objects.equals(path,"crabeLEFT")) {
            boucleDriveCrabe(DRIVE_SPEED, -inchValue, inchValue, 5.0);
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

    public void telemetryLoop(){

        telemetry.addData("status", "runtime: " + runtime.toString());
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

                    telemetry.addLine(String.format("\n==== (%d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
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

        if (showAimingTelemetry) {
            telemetry.addData("Servo Position", shooteraim.getPosition());
            telemetry.addData("viseurangle", viseurangle);
        }

        if (showGooberTelemetry) {
            telemetry.addData("Goober power", goober.getPower());

        }

        telemetry.update();
    };
}
