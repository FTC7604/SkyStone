package org.firstinspires.ftc.teamcode.Robot;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.BallisticMotionProfile;

import java.io.File;

import static java.lang.Math.*;

public class Robot {
    private BNO055IMU imu1 = null, imu2 = null;
    protected DcMotorEx rightFrontDriveMotor;
    protected DcMotorEx leftFrontDriveMotor;
    protected DcMotorEx rightBackDriveMotor;
    protected DcMotorEx leftBackDriveMotor;
    protected DcMotorEx rightIntakeMotor;
    protected DcMotorEx leftIntakeMotor;

    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private TouchSensor touchSensor;

    private HardwareMap hardwareMap;

    public Robot(OpMode opMode) {

        this.hardwareMap = opMode.hardwareMap;
        mapHardware();
    }

    private void mapHardware() {
        driveTrainHardwareMap();
        intakeHardwareMap();



        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");
        //imuHardwareMap();
    }

    private void driveTrainHardwareMap(){
        //drivetrain

        rightFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        leftFrontDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        rightBackDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        leftBackDriveMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");

        //sets the direction for the drivetrain
        rightFrontDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotorEx.Direction.FORWARD);

    }
    public void setDriveTrainZeroPowerProperty(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        leftFrontDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightFrontDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        leftBackDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
        rightBackDriveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setDriveTrainRunMode(DcMotor.RunMode runMode){
        leftFrontDriveMotor.setMode(runMode);
        rightFrontDriveMotor.setMode(runMode);
        leftBackDriveMotor.setMode(runMode);
        rightBackDriveMotor.setMode(runMode);
    }
    private void intakeHardwareMap(){
        //intake
        rightIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ri");
        leftIntakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "li");

        //sets the direction of the intake
        rightIntakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftIntakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        //so that it floats by at 0, could also be stop
        rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //methods that will serve us in the future, so we don't need to think about anything
    private void imuHardwareMap(){

    }
    private void colorSensorHardwareMap(){
        //gets the right sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "rcs");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "rds");
    }
    private void touchSensorHardwareMap(){
        touchSensor = hardwareMap.touchSensor.get("touch");
    }

    /**
     * All of the IMU methods that one could ever need, 0 is x, 1 is y, 2 is z:
     * Initialization and Calibration
     * Angle
     * Angular Velocity
     * Position
     * Velocity
     * Acceleration
     * Smoothed Acceleration
     * Temprature and Magnetic Field
     */
    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu1.initialize(parameters);
        imu2.initialize(parameters);

    }
    public void calibrateIMU(){
        BNO055IMU.CalibrationData calibrationData1 = imu1.readCalibrationData();
        File file1 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration1.json");
        ReadWriteFile.writeFile(file1, calibrationData1.serialize());

        BNO055IMU.CalibrationData calibrationData2 = imu2.readCalibrationData();
        File file2 = AppUtil.getInstance().getSettingsFile("AdafruitIMUCalibration2.json");
        ReadWriteFile.writeFile(file2, calibrationData2.serialize());
    }

    public double[] getBothIMUAngle(){
        return new double[]{
                (imu1.getAngularOrientation().secondAngle + imu2.getAngularOrientation().secondAngle) / 2,
                (imu1.getAngularOrientation().thirdAngle + imu2.getAngularOrientation().thirdAngle)   / 2,
                (imu1.getAngularOrientation().firstAngle + imu2.getAngularOrientation().firstAngle)   / 2,
        };
    }
    public double[] getRev2IMUAngle(){
        return new double[]{
                (imu1.getAngularOrientation().secondAngle) ,
                (imu1.getAngularOrientation().thirdAngle) ,
                (imu1.getAngularOrientation().firstAngle) ,
        };
    }
    public double[] getRev10IMUAngle(){
        return new double[]{
                (imu2.getAngularOrientation().secondAngle),
                (imu2.getAngularOrientation().thirdAngle),
                (imu2.getAngularOrientation().firstAngle),
        };
    }

    public double[] getIMUAnglularVelocity(){

        return new double[]{
                (imu1.getAngularVelocity().xRotationRate + imu2.getAngularVelocity().xRotationRate) / 2,
                (imu1.getAngularVelocity().yRotationRate + imu2.getAngularVelocity().yRotationRate )/ 2,
                (imu1.getAngularVelocity().zRotationRate + imu2.getAngularVelocity().zRotationRate) / 2,
        };
    }
    public double[] getIMUPositition(){

        return new double[]{
                (imu1.getPosition().x + imu2.getPosition().x) / 2,
                (imu1.getPosition().y + imu2.getPosition().y) / 2,
                (imu1.getPosition().z + imu2.getPosition().z) / 2,
        };
    }
    public double[] getIMUVelocity(){
        return new double[]{
                (imu1.getVelocity().xVeloc + imu2.getVelocity().xVeloc) / 2,
                (imu1.getVelocity().yVeloc + imu2.getVelocity().yVeloc) / 2,
                (imu1.getVelocity().zVeloc + imu2.getVelocity().zVeloc) / 2,
        };
    }
    public double[] getIMUAcceleration(){
        return new double[]{
                (imu1.getAcceleration().xAccel + imu2.getAcceleration().xAccel)/2,
                (imu1.getAcceleration().yAccel + imu2.getAcceleration().yAccel)/2,
                (imu1.getAcceleration().zAccel + imu2.getAcceleration().zAccel)/2,
        };
    }
    public double[] getIMULinearAcceleration(){
        return new double[]{
                (imu1.getLinearAcceleration().xAccel + imu2.getLinearAcceleration().xAccel)/2,
                (imu1.getLinearAcceleration().yAccel + imu2.getLinearAcceleration().yAccel)/2,
                (imu1.getLinearAcceleration().zAccel + imu2.getLinearAcceleration().zAccel)/2,
        };
    }

    /**
     * Everything that both color distance sensors could ever offer you:
     * Distance
     * ARGB, for objective look
     * RGB, for computation
     * HSV, for human look
     */
    public double getRightDistance(){
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }

    private double[] getARGB(ColorSensor colorSensor){
        return new double[]{
                (double) colorSensor.alpha() * 255,
                (double) colorSensor.red()   * 255,
                (double) colorSensor.green() * 255,
                (double) colorSensor.blue()  * 255
        };
    }
    private double[] getRGB(ColorSensor colorSensor){
        return new double[]{
                (double) colorSensor.red() * 255,
                (double) colorSensor.green() * 255,
                (double) colorSensor.blue() * 255
        };
    }
    private double[] getHSV(ColorSensor colorSensor){
        float[] floatHSV = new float[3];
        double[] doubleHSV = new double[3];

        //coverts the color that it gets into HSV
        Color.RGBToHSV(
                (int)((double)colorSensor.red() * 255),
                (int)((double)colorSensor.green() * 255),
                (int)((double)colorSensor.blue() * 255),

                floatHSV
        );

        //casts the doubles into floats.
        for(int i = 0; i < 3; i++) doubleHSV[i] = (double)floatHSV[i];

        return doubleHSV;

    }
    private double[] getARBG(){
        return (getARGB(colorSensor));
    }
    private double[] getRightRBG(){
        return (getRGB(colorSensor));
    }
    private double[] getHSV(){
        return (getHSV(colorSensor));
    }

    private boolean isPressed(){
        return touchSensor.isPressed();
    }


}
