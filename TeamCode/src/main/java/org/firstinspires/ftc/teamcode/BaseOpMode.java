package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
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

public class BaseOpMode extends LinearOpMode {
    protected final ElapsedTime runtime = new ElapsedTime();

    protected boolean showImuTelemetry = false;
    protected boolean showShooterTelemetry = false;
    protected boolean showColorTelemetry = false;
    protected boolean showGooberTelemetry = false;
    protected boolean showCameraTelemetry = false;

    protected AprilTagProcessor aprilTag;
    protected VisionPortal visionPortal;

    protected IMU imu;
    protected DcMotor frontLeftDrive;
    protected DcMotor frontRightDrive;
    protected DcMotor backLeftDrive;
    protected DcMotor backRightDrive;
    protected DcMotorEx shooter1;
    protected DcMotorEx shooter2;
    protected DcMotor goober;
    protected DcMotor goober2;
    protected NormalizedColorSensor sensor;

    protected final double SHOOTER_TICKS_PER_REV = 28;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     WHEEL_DISTANCE_INCHES   = 12.25;
    protected static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    protected static final double     INCHES_PER_DEGREE       = (WHEEL_DISTANCE_INCHES * Math.PI) / 360;
    protected static final double     DRIVE_SPEED             = 0.6;
    protected static final double     TURN_SPEED              = 0.5;
    static final double     APRILTAG_TOLERANCE_ANGLE= 5;
    static final double     APRILTAG_TOLERANCE_SIDE = 8;

    static final double     SHOOTER_P              = 90;
    static final double     SHOOTER_I              = 1.95;
    static final double     SHOOTER_D              = 0.1;
    static final double     SHOOTER_F              = 0;

    public static final int        CAMERA_RES_WIDTH       = 640;
    public static final int        CAMERA_RES_HEIGHT      = 480;

    boolean TELEOP = false;

    protected double shooterPower = 40; // Perfect speed to throw the ball when at the point of the launch triangle on the field
    protected double shooterTPS = shooterPower * SHOOTER_TICKS_PER_REV;

    protected String obelisk;
    protected String fieldSide;

    protected Map<Integer, String> obeliskPositions;
    protected Map<Integer, String> fieldSidePositions;


    @Override
    public void runOpMode(){

    }

    public void imuInit(){
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters params = new IMU.Parameters(orientationOnRobot);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(params);
        imu.resetYaw();
    }


    public void cameraInit() {

        showCameraTelemetry = true;

        obeliskPositions = new HashMap<Integer, String>();
        obeliskPositions.put(21,"GPP");
        obeliskPositions.put(22,"PGP");
        obeliskPositions.put(23,"PPG");


        fieldSidePositions = new HashMap<Integer, String>();
        fieldSidePositions.put(20,"blue");
        fieldSidePositions.put(24,"red");

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(1,2,3,4)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(CAMERA_RES_WIDTH,CAMERA_RES_HEIGHT));
        builder.enableLiveView(true);


        builder.addProcessor(aprilTag);

        // Create the vision portal the easy way.
        // if (USE_WEBCAM) {
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "webcam"), aprilTag);

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
        goober = hardwareMap.get(DcMotorEx.class, "goober");
        goober2 = hardwareMap.get(DcMotorEx.class, "goober2");
    }

    // TODO Finish BaseOpMode Refactor

    public void driveInit() {
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

        if (!TELEOP) {
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
    }

    public void telemetryInit(){
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    public void colorSensorInit() {
        showColorTelemetry = true;
        sensor = hardwareMap.get(NormalizedColorSensor.class,"colorsensor");
    }

    public void colorSensorLoop() {

    }

    public void shooterLoop() {
        shooterTPS = shooterPower * SHOOTER_TICKS_PER_REV;

        shooter1.setVelocity(gamepad2.right_trigger * shooterTPS);
        shooter2.setVelocity(gamepad2.right_trigger * shooterTPS);

        if (gamepad2.aWasPressed()) {
            if (shooterPower <= 0) {
                return;
            }
            shooterPower -= 1;
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
            if (shooterPower >= 100) {
                return;
            }
            shooterPower += 1;
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
            shooterPower = 42;
        }
    }

    public void gooberLoop() {

        // Change to Gamepad 1, Gamepad 2 is for testing purposes
        if (gamepad2.left_bumper) {
            goober.setPower(1);
            goober2.setPower(-1);
        } else if (gamepad2.right_bumper) {
            goober.setPower(-1);
            goober2.setPower(1);
        } else if (gamepad2.dpad_left) {
            goober.setPower(1);
            goober2.setPower(0);
        } else if (gamepad2.dpad_right) {
            goober.setPower(-1);
            goober2.setPower(0);
        } else if (gamepad2.dpad_up) {
            goober.setPower(0);
            goober2.setPower(1);
        } else if (gamepad2.dpad_down) {
            goober.setPower(0);
            goober2.setPower(-1);
        } else {
            goober.setPower(0);
            goober2.setPower(0);
        }

    }

    /*public void autoDriveFunction(boolean crabe, double speed, double leftInches, double rightInches, double timeout) {

    }*/

    public void aprilTagLoop() {

    }

    public void autoTelemetryLoop() {

    }

    public void telemetryLoop(){

        telemetry.addData("status", "runtime: " + runtime);
        if (!TELEOP) {
            autoTelemetryLoop();
        }

        if (showCameraTelemetry && !TELEOP) {
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

        if (showImuTelemetry) {
            AngleUnit unit = AngleUnit.DEGREES;
            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(unit));
            telemetry.addData("pitch", imu.getRobotYawPitchRollAngles().getPitch(unit));
            telemetry.addData("roll", imu.getRobotYawPitchRollAngles().getRoll(unit));

            telemetry.addData("angular velocity x", imu.getRobotAngularVelocity(unit).xRotationRate);
            telemetry.addData("angular velocity y", imu.getRobotAngularVelocity(unit).yRotationRate);
            telemetry.addData("angular velocity z", imu.getRobotAngularVelocity(unit).zRotationRate);
        }

        if (showShooterTelemetry) {
            telemetry.addData("left Shooter Velocity", shooter1.getVelocity());
            telemetry.addData("right Shooter Velocity", shooter2.getVelocity());
            telemetry.addData("shooter Power variable", shooterPower);
            telemetry.addData("shooter TPS (Supposed velocity)", shooterPower*SHOOTER_TICKS_PER_REV);
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

        telemetry.update();
    }
}
