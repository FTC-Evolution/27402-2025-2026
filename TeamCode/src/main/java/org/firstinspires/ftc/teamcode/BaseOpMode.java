package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

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
import org.firstinspires.ftc.teamcode.interfaces.Brain;
import org.firstinspires.ftc.teamcode.interfaces.Goober;
import org.firstinspires.ftc.teamcode.interfaces.Led;
import org.firstinspires.ftc.teamcode.interfaces.Shooter;
import org.firstinspires.ftc.teamcode.interfaces.Vision;
import org.firstinspires.ftc.teamcode.utility.SimplePID;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

class BaseOpMode extends LinearOpMode {

    protected final ElapsedTime runtime = new ElapsedTime();

    protected boolean showImuTelemetry = false;
    protected boolean showShooterTelemetry = false;
    protected boolean showColorTelemetry = false;
    protected boolean showGooberTelemetry = false;
    protected boolean showCameraTelemetry = false;

    protected DcMotor frontLeftDrive;
    protected DcMotor frontRightDrive;
    protected DcMotor backLeftDrive;
    protected DcMotor backRightDrive;

    protected Goober goober;

    protected Shooter shooter;

    protected Vision vision;

    SimplePID yawPID;
    SimplePID bearingPID;
    SimplePID rangePID;
    protected SimplePID robotOrientationPID;

    protected Brain brain;

    protected Led led;
    protected NormalizedColorSensor sensor;

    protected final double SHOOTER_TICKS_PER_REV = 28;
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double WHEEL_DISTANCE_INCHES = 12.25;
    protected static final double COUNTS_PER_INCH =
        (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
        (WHEEL_DIAMETER_INCHES * Math.PI);
    protected static final double INCHES_PER_DEGREE =
        (WHEEL_DISTANCE_INCHES * Math.PI) / 360;
    protected static final double DRIVE_SPEED = 0.6;
    protected static final double TURN_SPEED = 0.5;

    protected double shooterPower = Shooter.DEFAULT_SHOOTER_POWER; // Perfect speed to throw the ball when at the point of the launch triangle on the field
    protected double shooterTPS = shooterPower * SHOOTER_TICKS_PER_REV;

    public double degreesToInches(double degrees) {
        return (degrees / 360) * INCHES_PER_DEGREE;
    }

    @Override
    public void runOpMode() {}

    public void cameraInit() {
        vision = new Vision(hardwareMap.get(WebcamName.class, "Webcam 1"));
        showCameraTelemetry = true;

        yawPID = new SimplePID(0.03, 0, 0.002, -0.3, 0.3, 0.1);

        bearingPID = new SimplePID(0.025, 0.0, 0.001, -2, 2, 0.1);

        rangePID = new SimplePID(0.04, 0.0, 0.002, -0.5, 0.5, 2.0);

        robotOrientationPID = new SimplePID(0.03, 0, 0.002, -0.3, 0.3, 1.5);
    }

    public void shooterLedInit() {
        led = new Led(hardwareMap.get(Servo.class, "led"));
    }

    public void shooterInit() {
        showShooterTelemetry = true;

        shooter = new Shooter(
            hardwareMap.get(DcMotorEx.class, "leftshooter"),
            hardwareMap.get(DcMotorEx.class, "rightshooter")
        );
    }

    public void imuInit() {
        brain = new Brain(hardwareMap.get(IMU.class, "imu"));
    }

    public void gooberInit() {
        // showGooberTelemetry = true;

        goober = new Goober(
            hardwareMap.get(DcMotorEx.class, "goober"),
            hardwareMap.get(DcMotorEx.class, "goober2")
        );
    }

    public void driveInit() {
        // showImuTelemetry = true;
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

        autoDriveInitOverride();
    }

    public void autoDriveInitOverride() {}

    public void telemetryInit() {
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    /*public void colorSensorInit() {
        showColorTelemetry = true;
        sensor = hardwareMap.get(NormalizedColorSensor.class,"colorsensor");
    }

    public void colorSensorLoop() {

    }*/

    public void shooterLoop() {
        shooterTPS = shooter.speed(shooterPower);

        shooter.modVelocity(gamepad2.right_trigger * shooterTPS);

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
            if (
                shooterPower >=
                // 2147483647 // 32 bit integer limit :P
                45
            ) {
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
            shooterPower = Shooter.DEFAULT_SHOOTER_POWER;
        }
        if (gamepad2.xWasPressed()) {
            shooterPower = Shooter.DEFAULT_LONG_SHOOTER_POWER;
        }
        if (gamepad2.leftStickButtonWasPressed()) {
            shooterPower = Shooter.DEFAULT_SIDE_SHOOTER_POWER;
        }
        if (gamepad2.backWasPressed()) {
            shooterPower -= 0.5;
        }
        if (gamepad2.startWasPressed()) {
            shooterPower += 0.5;
        }
        if (gamepad1.backWasPressed()) {
            led.Confetti();
        }
        if (gamepad1.startWasPressed()) {
            led.TeamSpirit();
        }

        if (shooter.inReadyRange(shooterTPS)) {
            led.setColour(Led.Colour.GREEN);
            telemetry.addLine("Shooters Are Ready. GET SHOOTING.");
        } else if (
            (shooter.getVelocity()[0] > shooter.speed(shooterTPS) + 20 &&
                shooter.getVelocity()[1] > shooter.speed(shooterTPS) + 20) ||
            ((shooter.getVelocity()[0] < shooter.speed(shooterTPS) - 40 &&
                    shooter.getVelocity()[1] <
                    shooter.speed(shooterTPS) - 40) &&
                shooter.getVelocity()[0] != 0 &&
                shooter.getVelocity()[1] != 0 &&
                gamepad2.right_trigger != 0)
        ) {
            led.TeamSpiritBlitz();
            telemetry.addLine("Go BALL!");
        } else if (
            shooter.getVelocity()[0] > shooter.readyRange(shooterTPS)[0] &&
            shooter.getVelocity()[1] > shooter.readyRange(shooterTPS)[1]
        ) {
            led.setColour(Led.Colour.RED);
            telemetry.addLine("Overshooting speed - Restart SHOOTER!");
        } else {
            led.setColour(Led.Colour.WHITE);
            telemetry.addLine("Not Fast Enough. WAIT UP.");
        }
    }

    public void gooberLoop() {
        // Change to Gamepad 1, Gamepad 2 is for testing purposes
        if (gamepad2.left_bumper) {
            goober.modPower(1);
        } else if (gamepad2.right_bumper) {
            goober.modPower(-1);
        } else if (gamepad2.dpad_left) {
            goober.modSoloPower(1, Goober.Type.TOP);
        } else if (gamepad2.dpad_right) {
            goober.modSoloPower(-1, Goober.Type.TOP);
        } else if (gamepad2.dpad_up) {
            goober.modSoloPower(1, Goober.Type.BOTTOM);
        } else if (gamepad2.dpad_down) {
            goober.modSoloPower(-1, Goober.Type.BOTTOM);
        } else {
            goober.modPower(0);
        }
    }

    /*public void autoDriveFunction(boolean crabe, double speed, double leftInches, double rightInches, double timeout) {

    }*/

    public void autoTelemetryLoop() {}

    public boolean alignAprilTag(
        double target_distance,
        Vision.UpdateGoalAprilTagGoal goalDetect,
        double yaw_offset
    ) {
        vision.updateGoalAprilTag(goalDetect);

        double turn = yawPID.update(vision.getYawError(yaw_offset));
        double strafe = bearingPID.update(vision.getHeadingError() * 2);
        double drive = rangePID.update(vision.getRangeError(target_distance));

        telemetry.addData("Target distance", target_distance);

        telemetry.addLine("--------errorsrreturned--------");

        telemetry.addData("Drive", drive);
        telemetry.addData("strafe", strafe);
        telemetry.addData("turn", turn);

        telemetry.addLine("-------------Error passed-----------");
        telemetry.addData("yaw", vision.getYawError(yaw_offset));
        telemetry.addData("heading", vision.getHeadingError() * 2);
        telemetry.addData("range", vision.getRangeError(target_distance));

        telemetry.update();

        /* drive = vision.getRangeError(target_distance);
        strafe = vision.getHeadingError();
        turn = vision.getYawError(); */

        if (Math.abs(turn) > 5) {
            strafe = 0;
            drive = 0;
        }

        moveRobot(drive, strafe, turn);

        return turn == 0 && strafe == 0 && drive == 0;
    }

    public boolean rotateRobot(double target_rotation) {
        double turn = robotOrientationPID.update(
            AngleUnit.normalizeDegrees(
                target_rotation - brain.getYaw(AngleUnit.DEGREES)
            )
        );

        telemetry.addData(
            "Target error passed to update",
            AngleUnit.normalizeDegrees(
                target_rotation - brain.getYaw(AngleUnit.DEGREES)
            )
        );
        telemetry.addData("Current yaw", brain.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Target rotate", target_rotation);

        telemetry.addData(
            "Returned rotation error (.error)",
            robotOrientationPID.error
        );
        telemetry.addData("Current rotation error (.update return)", turn);
        telemetry.update();

        moveRobot(0, 0, turn);

        return turn == 0;
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower = x - y - yaw;
        double frontRightPower = x + y + yaw;
        double backLeftPower = x + y - yaw;
        double backRightPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(
            Math.abs(frontLeftPower),
            Math.abs(frontRightPower)
        );
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    @SuppressLint("DefaultLocale")
    public void telemetryLoop() {
        telemetry.addData("status", "runtime: " + runtime);
        autoTelemetryLoop();

        if (showCameraTelemetry) {
            // AprilTagDetection detection = vision.lastSeenGoal;
            AprilTagMetadata detectionMetadata = vision.getFieldGoalMetadata();
            AprilTagPoseFtc detectionPose = vision.getFieldGoalFtcPose();
            // Step through the list of detections and display info for each one.
            if (detectionMetadata != null && detectionPose != null) {
                telemetry.addLine(
                    String.format(
                        "\n==== (%d) %s",
                        detectionMetadata.id,
                        detectionMetadata
                    )
                );
                telemetry.addLine(
                    String.format(
                        "XYZ %6.1f %6.1f %6.1f  (inch)",
                        detectionPose.x,
                        detectionPose.y,
                        detectionPose.z
                    )
                );
                telemetry.addLine(
                    String.format(
                        "PRY %6.1f %6.1f %6.1f  (deg)",
                        detectionPose.pitch,
                        detectionPose.roll,
                        detectionPose.yaw
                    )
                );
                telemetry.addLine(
                    String.format(
                        "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
                        detectionPose.range,
                        detectionPose.bearing,
                        detectionPose.elevation
                    )
                );
            }

            // Add "key" information to telemetry
            telemetry.addLine(
                "key:\nXYZ = X (Right), Y (Forward), Z (Up) dist."
            );
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
            telemetry.addData("Yaw error", yawPID.error);
            telemetry.addData("Strafe error", bearingPID.error);

            telemetry.addData("Last seen Pattern", vision.obeliskName);
            telemetry.addData("Last seen Target", vision.fieldGoalName);
        }

        if (showImuTelemetry) {
            AngleUnit unit = AngleUnit.DEGREES;
            telemetry.addData("yaw", brain.getYaw(unit));
            telemetry.addData("pitch", brain.getPitch(unit));
            telemetry.addData("roll", brain.getRoll(unit));
        }

        if (showShooterTelemetry) {
            telemetry.addData(
                "left Shooter Velocity",
                shooter.getVelocity()[0]
            );
            telemetry.addData(
                "right Shooter Velocity",
                shooter.getVelocity()[1]
            );
            telemetry.addData("shooter Power variable", shooterPower);
            telemetry.addData(
                "shooter TPS (Supposed velocity)",
                shooterPower * SHOOTER_TICKS_PER_REV
            );
        }

        if (showColorTelemetry) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            int colorCode = colors.toColor();
            String ballColor;

            telemetry.addData("Hue", JavaUtil.colorToHue(colorCode));
            telemetry.addData(
                "Saturation",
                "%.3f",
                JavaUtil.colorToSaturation(colorCode)
            );
            telemetry.addData(
                "Value",
                "%.3f",
                JavaUtil.colorToValue(colorCode)
            );
            telemetry.addData("Color", JavaUtil.colorToText(colorCode));

            telemetry.addData("Alpha", "%.3f", colors.alpha);

            telemetry.addData(
                "Light level",
                "%.3f",
                ((OpticalDistanceSensor) sensor).getLightDetected()
            );

            if (
                (JavaUtil.colorToHue(colorCode) >= 200) &&
                (JavaUtil.colorToHue(colorCode) <= 300)
            ) {
                ballColor = "purple";
            } else if (
                (JavaUtil.colorToHue(colorCode) >= 30) &&
                (JavaUtil.colorToHue(colorCode) <= 199)
            ) {
                ballColor = "green";
            } else {
                ballColor = "unknown";
            }

            telemetry.addData("Ball color", ballColor);
        }

        if (showGooberTelemetry) {
            telemetry.addData(
                "top Goober power",
                goober.getPower().get(Goober.Type.TOP)
            );
            telemetry.addData(
                "bottom Goober power",
                goober.getPower().get(Goober.Type.BOTTOM)
            );
        }

        telemetry.update();
    }
}
