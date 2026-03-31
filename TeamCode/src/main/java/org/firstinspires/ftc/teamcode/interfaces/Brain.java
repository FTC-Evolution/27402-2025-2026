package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Brain {
    protected final IMU imu;

    public Brain(IMU imu) {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        IMU.Parameters params = new IMU.Parameters(orientationOnRobot);

        this.imu = imu;

        this.imu.initialize(params);
        this.imu.resetYaw();
    }

    public double getYaw(AngleUnit unit) {
        return this.imu.getRobotYawPitchRollAngles().getYaw(unit);
    }
    public double getPitch(AngleUnit unit) {
        return this.imu.getRobotYawPitchRollAngles().getPitch(unit);
    }
    public double getRoll(AngleUnit unit) {
        return this.imu.getRobotYawPitchRollAngles().getRoll(unit);
    }

    public void resetYaw() {
        this.imu.resetYaw();
    }
}
