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

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    final double SHOOTER_TICKS_PER_REV = 28;


    private DcMotor goober;
    private DcMotor goober2;
    private NormalizedColorSensor sensor;
    private Servo shooteraim;

    private ElapsedTime runtime = new ElapsedTime();

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




    @Override
    public void runOpMode() {

        InitDrive();
        shooterInit();
        aimInit();
        gooberInit();
        // colorSensorInit();
        telemetryInit();
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // buttonLoop();
            boucleDrive();
            aimLoop();
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

    public void aimInit() {
        showAimingTelemetry = true;
        shooteraim = hardwareMap.get(Servo.class, "aimservo");
        shooteraim.setPosition(viseurangle/180);
    }

    public void aimLoop() {
        if (gamepad2.dpadUpWasPressed()){
            viseurangle += 5;
        } else if (gamepad2.dpadDownWasPressed()) {
            viseurangle -= 5;
        }

        shooteraim.setPosition(viseurangle/180);
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
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        backLeftDrive.setPower(backLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backRightDrive.setPower(backRightPower);
		//TODO ajouter le code pour le field orienter. Le code si dessous est celui de gobilda
		/*
		double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
		*/
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
    };
}
