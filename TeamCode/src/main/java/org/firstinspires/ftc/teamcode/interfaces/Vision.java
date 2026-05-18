package org.firstinspires.ftc.teamcode.interfaces;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Vision {
    public static final int        CAMERA_RES_WIDTH       = 640;
    public static final int        CAMERA_RES_HEIGHT      = 480;

    public static final double        CAMERA_X               = 10;
    public static final double        CAMERA_Y               = 0;
    public static final double        CAMERA_Z               = 0;

    public static final double        CAMERA_YAW             = 0;
    public static final double        CAMERA_PITCH           = -80;
    public static final double        CAMERA_ROLL            = 0;

    public static final double     APRILTAG_TOLERANCE_DISTANCE = 3;
    public static final double     APRILTAG_TOLERANCE_ANGLE= 5;
    public static final double     APRILTAG_TOLERANCE_SIDE = 8;

    public String obeliskName;
    public String fieldGoalName;

    public AprilTagDetection lastSeenGoal = null;
    public AprilTagDetection currentGoal = null;

    Map<Integer, String> obeliskPositions = new HashMap<>();
    Map<Integer, String> fieldSidePositions = new HashMap<>();



    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;
    public Vision(WebcamName camera) {

        Position cameraPosition = new Position(DistanceUnit.INCH,
                CAMERA_X,
                CAMERA_Y,
                CAMERA_Z, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                CAMERA_YAW,
                CAMERA_PITCH,
                CAMERA_ROLL, 0);

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();

        visionPortalBuilder.setCamera(camera);
        visionPortalBuilder.setCameraResolution(new Size(CAMERA_RES_WIDTH, CAMERA_RES_HEIGHT));
        visionPortalBuilder.enableLiveView(true);

        visionPortalBuilder.addProcessor(aprilTagProcessor);

        visionPortal = visionPortalBuilder.build();

        // init hashmaps

        obeliskPositions.put(21,"GPP");
        obeliskPositions.put(22,"PGP");
        obeliskPositions.put(23,"PPG");

        fieldSidePositions.put(20,"blue");
        fieldSidePositions.put(24,"red");
    }

    public List<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    public static enum UpdateGoalAprilTagGoal {
        BOTH,
        RED,
        BLUE
    }

    public void updateGoalAprilTag(UpdateGoalAprilTagGoal goalDetect) {
        currentGoal = null;
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection.metadata != null) {
                    boolean detectionCondition=false;
                    switch (goalDetect) {
                    case RED:
                        detectionCondition =  detection.id == 24;
                        break;
                    case BLUE:
                        detectionCondition = detection.id == 20;
                        break;
                    case BOTH:
                        detectionCondition = detection.id == 20 || detection.id == 24;
                        break;
                };


                if (detectionCondition) { // GOAL april ids
                    fieldGoalName = fieldSidePositions.get(detection.id);
                    lastSeenGoal = detection;
                    currentGoal = detection;
                    break;
                }
            }
        }

    }

    public double getFieldGoalDistance() {
        if (currentGoal != null) {
            return currentGoal.ftcPose.range;
        } else { return 0.0; }
    }

    public double absoluteFieldGoalDistance() {
        return Math.abs(getFieldGoalDistance());
    }

    public double getFieldGoalSideDistance() {
        if (currentGoal != null) {
            return currentGoal.ftcPose.x;
        } else { return 0.0; }
    }

    public double getFieldGoalBearing() {
        if (currentGoal != null) {
            return currentGoal.ftcPose.bearing;
        } else { return 0.0; }
    }

    public double getFieldGoalYaw(double offset) {
        if (currentGoal != null) {
            return currentGoal.ftcPose.yaw - offset;
        } else { return 0.0; }
    }

    public AprilTagMetadata getFieldGoalMetadata() {
        if (currentGoal != null) {
            return currentGoal.metadata;
        } else {
            return null;
        }
    }

    public AprilTagPoseFtc getFieldGoalFtcPose() {
        if (currentGoal != null) {
            return currentGoal.ftcPose;
        } else {
            return null;
        }
    }

    public double getRangeError(double target_distance) {
        return absoluteFieldGoalDistance() - target_distance;
    }

    public double getHeadingError() {
        return getFieldGoalBearing();
    }

    public double getYawError(double offset) {
        return getFieldGoalYaw(offset);
    }
}