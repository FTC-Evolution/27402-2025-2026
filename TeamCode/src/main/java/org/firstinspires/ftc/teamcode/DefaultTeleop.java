package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="Teleop des Sec 2 27402")

public class DefaultTeleop extends LinearOpMode {
    boolean showImuTelemetry = false;
    boolean showShooterTelemetry = false;
    boolean showColorTelemetry = false;
    boolean showAimingTelemetry = false;
    boolean showGooberTelemetry = false;
    boolean showCameraTelemetry = false;

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    final double SHOOTER_TICKS_PER_REV = 28;


    private DcMotor goober;
    private DcMotor goober2;
    private NormalizedColorSensor sensor;
    private Servo shooteraim;

    private final ElapsedTime runtime = new ElapsedTime();

    private double shooterPower = 0.5;
    double shooterTPS = shooterPower * SHOOTER_TICKS_PER_REV;

    double counter1 = 0;
    double counter2 = 0;

    private String obelisk;
    private String fieldSide;

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

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


    // This declares the IMU needed to get the current direction the robot is faci ng
    IMU imu;
    AngleUnit angle = AngleUnit.DEGREES;
    IMU.Parameters imup = new IMU.Parameters(orientationOnRobot);

    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_DISTANCE_INCHES   = 12.25;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     INCHES_PER_DEGREE       = (WHEEL_DISTANCE_INCHES * Math.PI) / 360;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;




    @Override
    public void runOpMode() {

        InitDrive();
        shooterInit();
        gooberInit();
        // colorSensorInit();
        telemetryInit();
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // buttonLoop();
            boucleDrive();
            shooterLoop();
            gooberLoop();
            // colorSensorLoop();
            telemetryLoop();
        }

    }

    public void shooterInit() {
        showShooterTelemetry = true;
        shooter1 = hardwareMap.get(DcMotorEx.class, "leftshooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "rightshooter");
    }

    public void gooberInit(){
        showGooberTelemetry = true;
        goober = hardwareMap.get(DcMotorEx.class, "goober");
        goober2 = hardwareMap.get(DcMotorEx.class, "goober2");

    }

    public void gooberLoop(){

        // Change to Gamepad 1, Gamepad 2 is for testing purposes
        if (gamepad2.dpad_left) {
            goober.setPower(1);
            goober2.setPower(-1);
        } else if (gamepad2.dpad_right) {
            goober.setPower(-1);
            goober2.setPower(1);
        } else {
            goober.setPower(0);
            goober2.setPower(0);
        }

    }
    public void shooterLoop() {

        shooterTPS = shooterPower * SHOOTER_TICKS_PER_REV;

        shooter1.setVelocity(-gamepad2.right_trigger * shooterTPS);
        shooter2.setVelocity(gamepad2.right_trigger * shooterTPS);

        if (gamepad2.xWasPressed()) {
            if (shooterPower <= 1.0){ return; }
            shooterPower -= 0.05;
        }
        if (gamepad2.yWasPressed()) {
            if (shooterPower >= 1.0){ return; }
            if (shooterPower == 0.95) {
                shooterPower += 0.05;
                return;
            }
            shooterPower += 0.1;
        }
        if (gamepad2.bWasPressed()) {
            if (shooterPower >= 0.0){ return; }
            shooterPower += 0.05;
        }
        if (gamepad2.aWasPressed()) {
            if (shooterPower <= 0.0){ return; }
            if (shooterPower == 0.05) {
                shooterPower -= 0.05;
            }
            shooterPower -= 0.1;
        }
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

                    if (gamepad1.leftBumperWasPressed()) {
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
    }

    public void InitDrive() {
        showImuTelemetry = true;
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftDrive = hardwareMap.dcMotor.get("fld");
        backLeftDrive = hardwareMap.dcMotor.get("bld");
		frontRightDrive = hardwareMap.dcMotor.get("frd");
        backRightDrive = hardwareMap.dcMotor.get("brd");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void boucleDrive() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x*Math.cos(-botDirection) - y * Math.sin(-botDirection);
        double rotY = x*Math.sin(-botDirection) + y * Math.cos(-botDirection);

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
    public void buttoninnit() {


    }

    public void buttonLoop() {


        // telemetry.addLine("get 50 to win");

        if (end)
        {
            return;
        }
        if (gamepad1.xWasPressed()) {
            counter1 +=1;
        }

        if (gamepad2.xWasPressed()) {
            counter2 +=1;
        }


        else if (counter1 >= 50) {
            winner = "player 1";
            end = true;

        }
        else if (counter2 >= 50) {
            winner = "player 2";
            end = true;
        }

        telemetry.addData("player 1 score", counter1);
        telemetry.addData("player 2 score", counter2);
        telemetry.addData("winner", winner);
        telemetry.addData("game over", end);

        telemetry.update();
    }

    public void colorSensorInit() {
        showColorTelemetry = true;
        sensor = hardwareMap.get(NormalizedColorSensor.class,"colorsensor");
    }
    public void colorSensorLoop(){

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

        if (showImuTelemetry) {
            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(angle));
            telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(angle));
            telemetry.addData("roll", imu.getRobotYawPitchRollAngles().getRoll(angle));

            telemetry.addData("angular velocity x", imu.getRobotAngularVelocity(angle).xRotationRate);
            telemetry.addData("angular velocity y", imu.getRobotAngularVelocity(angle).yRotationRate);
            telemetry.addData("angular velocity z", imu.getRobotAngularVelocity(angle).zRotationRate);
        }

        if (showShooterTelemetry) {
            telemetry.addData("left Shooter power", shooter1.getPower());
            telemetry.addData("right Shooter power", shooter2.getPower());
            telemetry.addData("motor shooterPower", shooterPower);
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
    }
}

//jku./