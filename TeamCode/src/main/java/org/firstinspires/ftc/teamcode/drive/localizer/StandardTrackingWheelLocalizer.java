package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 * Nevermind, it's two wheel now
 */
@Config
public class StandardTrackingWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.944882; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    private ExpansionHubMotor frontEncoder, strafeEncoder;
    private BNO055IMU imu;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, BNO055IMU imu) {
        super(Arrays.asList(
                new Pose2d(-1.75, -1.2, 0), // front
                new Pose2d(-1.625, 1.5, Math.toRadians(90)) // strafe
        ));

        frontEncoder = hardwareMap.get(ExpansionHubMotor.class, "ri");
        strafeEncoder = hardwareMap.get(ExpansionHubMotor.class, "li");
        this.imu = imu;
    }

    @NonNull
    @Override
    public double getHeading(){
        return imu.getAngularOrientation().firstAngle;
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        //Optimized (Requires you to get the hub from the drive, though)
        /*RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>(2);
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(frontEncoder)));
        wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(strafeEncoder)));*/

        //Unoptimized
        List<Double> wheelPositions = Arrays.asList(
            encoderTicksToInches(frontEncoder.getCurrentPosition()),
            encoderTicksToInches(strafeEncoder.getCurrentPosition())
        );

        return wheelPositions;
    }
}
