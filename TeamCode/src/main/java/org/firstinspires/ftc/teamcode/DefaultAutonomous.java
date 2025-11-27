package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="Mode Autonome Sec 2 27402")
public class DefaultAutonomous {
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

    public void runOpMode() {

        // InitDrive();
        // shooterInit();
        // aimInit();
        // buttoninnit();
        // gooberInit();
        initAprilTag();
        colorSensorInit();
        telemetryInit();
        runtime.reset();

        // buttonLoop();
        // boucleDrive();
        // aimLoop();
        // shooterLoop();
        // gooberLoop();
        aprilTagLoop();
        colorSensorLoop();
        telemetryLoop();

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

        //visionPortal = VisionPortal.easyCreateWithDefaults(
        //        BuiltinCameraDirection.BACK, aprilTag);
        // }

    }

    public void aprilTagLoop(){
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection.metadata != null) {
                if (detection.metadata.name.contains("Obelisk")) {
                    obelisk = obeliskPositions.get(detection.id);
                } else {
                    fieldSide = fieldSidePositions.get(detection.id);
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
        if (gamepad2.dpadUpWasPressed()){
            viseurangle += 5;
        } else if (gamepad2.dpadDownWasPressed()) {
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
        if (gamepad1.right_trigger >= 0.5) {
            goober.setPower(-1);
        } else if (gamepad1.left_trigger >= 0.5) {
            goober.setPower(1);
        } else {
            goober.setPower(0);
        }

    }
    public void shooterLoop() {

        shooter1.setPower(gamepad2.right_trigger / divider);
        shooter2.setPower(-gamepad2.right_trigger / divider);

        shooter1.setPower(-gamepad2.left_trigger / divider);
        shooter2.setPower(gamepad2.left_trigger / divider);

        if (gamepad2.x) {
            divider = 1;

        }
        if (gamepad2.y) {
            divider = 2;
        }
        if (gamepad2.b) {
            divider = 3;
        }
        if (gamepad2.a) {
            divider = 4;
        }
    }





    public void InitDrive() {
        showImuTelemetry = true;
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftDrive = hardwareMap.dcMotor.get("motor0");
        backLeftDrive = hardwareMap.dcMotor.get("motor2");
        frontRightDrive = hardwareMap.dcMotor.get("motor1");
        backRightDrive = hardwareMap.dcMotor.get("motor3");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
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
        sensor = hardwareMap.get(NormalizedColorSensor.class,"sensor");
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
