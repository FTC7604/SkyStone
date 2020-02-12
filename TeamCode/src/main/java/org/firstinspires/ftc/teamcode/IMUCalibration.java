package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;

import static com.qualcomm.hardware.bosch.BNO055IMU.Register.*;

//TODO: calibrate the imus
//TODO: Y to see if it decalibrates the imus (it loads a blank file).
//TODO: X to see if it saves the imus and can load them upon reinit.

//TODO: Troubleshoot.

@TeleOp(name = "IMU Calibration", group = "Sensor")
public class IMUCalibration extends LinearOpMode {

    private BNO055IMU imu1;
    private BNO055IMU imu2;



    @Override
    public void runOpMode(){


        initIMUs();

        // Wait until we're told to go
        while(!isStarted()){
            idle();
        }

        while(opModeIsActive()){

            if (gamepad1.x) {
                saveCalibration();
                while(gamepad1.x){
                    idle();
                }
            }
            else if (gamepad1.y) {
                loadCalibration();
                while(gamepad1.y){
                    idle();
                }
            }
            else {


                if (gamepad1.dpad_up) {
                    telemetry.addData("OPR_MODE 1", imu1.read8(BNO055IMU.Register.OPR_MODE));
                    telemetry.addData("PWR_MODE 1", imu1.read8(BNO055IMU.Register.PWR_MODE));
                    telemetry.addData("OPR_MODE 2", imu2.read8(BNO055IMU.Register.OPR_MODE));
                    telemetry.addData("PWR_MODE 2", imu2.read8(BNO055IMU.Register.PWR_MODE));
                }
                else if (gamepad1.dpad_right) {
                    telemetry.addData("ACC X1", imu1.getLinearAcceleration().xAccel);
                    telemetry.addData("ACC Y1", imu1.getLinearAcceleration().yAccel);
                    telemetry.addData("ACC Z1", imu1.getLinearAcceleration().zAccel);

                    telemetry.addData("ANG F1", imu1.getAngularOrientation().firstAngle);
                    telemetry.addData("ANG S1", imu1.getAngularOrientation().secondAngle);
                    telemetry.addData("ANG T1", imu1.getAngularOrientation().thirdAngle);

                    telemetry.addData("ACC X2", imu2.getLinearAcceleration().xAccel);
                    telemetry.addData("ACC Y2", imu2.getLinearAcceleration().yAccel);
                    telemetry.addData("ACC Z2", imu2.getLinearAcceleration().zAccel);

                    telemetry.addData("ANG F2", imu2.getAngularOrientation().firstAngle);
                    telemetry.addData("ANG S2", imu2.getAngularOrientation().secondAngle);
                    telemetry.addData("ANG T2", imu2.getAngularOrientation().thirdAngle);

                }
                else if (gamepad1.dpad_left) {
                    telemetry.addData("ACC X1", imu1.getLinearAcceleration().xAccel);
                    telemetry.addData("ACC Y1", imu1.getLinearAcceleration().yAccel);
                    //                    telemetry.addData("ACC Z1", imu1.getLinearAcceleration().zAccel);

                    telemetry.addData("ANG F1", imu1.getAngularOrientation().firstAngle);
                    //                    telemetry.addData("ANG S1", imu1.getAngularOrientation().secondAngle);
                    //                    telemetry.addData("ANG T1", imu1.getAngularOrientation().thirdAngle);

                    telemetry.addData("ACC X2", imu2.getLinearAcceleration().xAccel);
                    telemetry.addData("ACC Y2", imu2.getLinearAcceleration().yAccel);
                    //                    telemetry.addData("ACC Z2", imu2.getLinearAcceleration().zAccel);

                    telemetry.addData("ANG F2", imu2.getAngularOrientation().firstAngle);
                    //                    telemetry.addData("ANG S2", imu2.getAngularOrientation().secondAngle);
                    //                    telemetry.addData("ANG T2", imu2.getAngularOrientation().thirdAngle);

                }
                else {
                    boolean[] calibrationStatus1 = byteToBooleanArray(imu1.read8(BNO055IMU.Register.CALIB_STAT));
                    boolean[] calibrationStatus2 = byteToBooleanArray(imu2.read8(BNO055IMU.Register.CALIB_STAT));

                    telemetry.addData("SYS 1", calibrationStatus(calibrationStatus1[0], calibrationStatus1[1]));
                    telemetry.addData("GYR 1", calibrationStatus(calibrationStatus1[2], calibrationStatus1[3]));
                    telemetry.addData("ACC 1", calibrationStatus(calibrationStatus1[4], calibrationStatus1[5]));
                    telemetry.addData("MAG 1", calibrationStatus(calibrationStatus1[6], calibrationStatus1[7]));

                    telemetry.addData("SYS 2", calibrationStatus(calibrationStatus2[0], calibrationStatus2[1]));
                    telemetry.addData("GYR 2", calibrationStatus(calibrationStatus2[2], calibrationStatus2[3]));
                    telemetry.addData("ACC 2", calibrationStatus(calibrationStatus2[4], calibrationStatus2[5]));
                    telemetry.addData("MAG 2", calibrationStatus(calibrationStatus2[6], calibrationStatus2[7]));
                }
            }
            telemetry.update();
        }
    }

    void initIMUs(){
        imu1 = hardwareMap.get(BNO055IMU.class, "imu");
        imu2 = hardwareMap.get(BNO055IMU.class, "imu 1");

        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();

        parameters1.mode = BNO055IMU.SensorMode.NDOF;
        parameters2.mode = BNO055IMU.SensorMode.NDOF;

        imu1.initialize(parameters1);
        imu2.initialize(parameters2);
    }

    /**
     * takes the string commands from the datasheet (which is designed for c) and implements them
     * for the FTC object BNO055IMU
     *
     * @param register the register that will be changed
     * @param command  the string command as described in the data book
     */
    void partialWriteIMU1(BNO055IMU.Register register, String command){
        //reads in the input as boolean array
        byte      input = imu1.read8(register);
        boolean[] value = byteToBooleanArray(input);

        //iterates though the whole array and overwrites it if the command demmands it (if it is 0 or 1)
        for (int i = 0; i < 8; i++) {
            if (command.charAt(i) == (char) 48) value[i] = false;
            if (command.charAt(i) == (char) 49) value[i] = true;
        }

        //writes the value back to the first imu
        byte output = booleanArrayToByte(value);
        imu1.write8(register, output);

    }

    /**
     * takes the string commands from the datasheet (which is designed for c) and implements them
     * for the FTC object BNO055IMU
     *
     * @param register the register that will be changed
     * @param command  the string command as described in the data book
     */
    void partialWriteIMU2(BNO055IMU.Register register, String command){
        //reads in the input as boolean array
        byte      input = imu2.read8(register);
        boolean[] value = byteToBooleanArray(input);

        //iterates though the whole array and overwrites it if the command demmands it (if it is 0 or 1)
        for (int i = 0; i < 8; i++) {
            if (command.charAt(i) == (char) 48) value[i] = false;
            if (command.charAt(i) == (char) 49) value[i] = true;
        }

        //writes the value back to the first imu
        byte output = booleanArrayToByte(value);
        imu2.write8(register, output);

    }

    private byte booleanArrayToByte(boolean[] booleanArray){
        byte oneByte    = 0;
        int  multiplier = 1;
        for (int i = 0; i < 8; i++) {
            if (booleanArray[7 - i]) oneByte += multiplier;
            multiplier *= 2;
        }

        return oneByte;
    }

    private boolean[] byteToBooleanArray(byte b){
        boolean[] bs = new boolean[8];
        for (int i = 0; i < 8; i++) {
            bs[i] = ((b >> 7 - i) & 1) == 1;
        }
        return bs;
    }

    private int calibrationStatus(boolean firstBit, boolean secondBit){
        int calibrationStatus = 0;

        if (firstBit) calibrationStatus += 2;
        if (secondBit) calibrationStatus += 1;

        return calibrationStatus;
    }

    /**
     * reads all of the calibation data from the imu and loads it onto the properties file
     */
    private void saveCalibration(){

        byte ACC_OFFSET_X_LSB_1 = imu1.read8(ACC_OFFSET_X_LSB);
        byte ACC_OFFSET_X_MSB_1 = imu1.read8(ACC_OFFSET_X_MSB);
        byte ACC_OFFSET_Y_LSB_1 = imu1.read8(ACC_OFFSET_Y_LSB);
        byte ACC_OFFSET_Y_MSB_1 = imu1.read8(ACC_OFFSET_Y_MSB);
        byte ACC_OFFSET_Z_LSB_1 = imu1.read8(ACC_OFFSET_Z_LSB);
        byte ACC_OFFSET_Z_MSB_1 = imu1.read8(ACC_OFFSET_Z_MSB);

        byte MAG_OFFSET_X_LSB_1 = imu1.read8(MAG_OFFSET_X_LSB);
        byte MAG_OFFSET_X_MSB_1 = imu1.read8(MAG_OFFSET_X_MSB);
        byte MAG_OFFSET_Y_LSB_1 = imu1.read8(MAG_OFFSET_Y_LSB);
        byte MAG_OFFSET_Y_MSB_1 = imu1.read8(MAG_OFFSET_Y_MSB);
        byte MAG_OFFSET_Z_LSB_1 = imu1.read8(MAG_OFFSET_Z_LSB);
        byte MAG_OFFSET_Z_MSB_1 = imu1.read8(MAG_OFFSET_Z_MSB);

        byte GYR_OFFSET_X_LSB_1 = imu1.read8(GYR_OFFSET_X_LSB);
        byte GYR_OFFSET_X_MSB_1 = imu1.read8(GYR_OFFSET_X_MSB);
        byte GYR_OFFSET_Y_LSB_1 = imu1.read8(GYR_OFFSET_Y_LSB);
        byte GYR_OFFSET_Y_MSB_1 = imu1.read8(GYR_OFFSET_Y_MSB);
        byte GYR_OFFSET_Z_LSB_1 = imu1.read8(GYR_OFFSET_Z_LSB);
        byte GYR_OFFSET_Z_MSB_1 = imu1.read8(GYR_OFFSET_Z_MSB);

        byte ACC_RADIUS_LSB_1 = imu1.read8(ACC_RADIUS_LSB);
        byte ACC_RADIUS_MSB_1 = imu1.read8(ACC_RADIUS_MSB);
        byte MAG_RADIUS_LSB_1 = imu1.read8(MAG_RADIUS_LSB);
        byte MAG_RADIUS_MSB_1 = imu1.read8(MAG_RADIUS_MSB);

        byte ACC_OFFSET_X_LSB_2 = imu2.read8(ACC_OFFSET_X_LSB);
        byte ACC_OFFSET_X_MSB_2 = imu2.read8(ACC_OFFSET_X_MSB);
        byte ACC_OFFSET_Y_LSB_2 = imu2.read8(ACC_OFFSET_Y_LSB);
        byte ACC_OFFSET_Y_MSB_2 = imu2.read8(ACC_OFFSET_Y_MSB);
        byte ACC_OFFSET_Z_LSB_2 = imu2.read8(ACC_OFFSET_Z_LSB);
        byte ACC_OFFSET_Z_MSB_2 = imu2.read8(ACC_OFFSET_Z_MSB);

        byte MAG_OFFSET_X_LSB_2 = imu2.read8(MAG_OFFSET_X_LSB);
        byte MAG_OFFSET_X_MSB_2 = imu2.read8(MAG_OFFSET_X_MSB);
        byte MAG_OFFSET_Y_LSB_2 = imu2.read8(MAG_OFFSET_Y_LSB);
        byte MAG_OFFSET_Y_MSB_2 = imu2.read8(MAG_OFFSET_Y_MSB);
        byte MAG_OFFSET_Z_LSB_2 = imu2.read8(MAG_OFFSET_Z_LSB);
        byte MAG_OFFSET_Z_MSB_2 = imu2.read8(MAG_OFFSET_Z_MSB);

        byte GYR_OFFSET_X_LSB_2 = imu2.read8(GYR_OFFSET_X_LSB);
        byte GYR_OFFSET_X_MSB_2 = imu2.read8(GYR_OFFSET_X_MSB);
        byte GYR_OFFSET_Y_LSB_2 = imu2.read8(GYR_OFFSET_Y_LSB);
        byte GYR_OFFSET_Y_MSB_2 = imu2.read8(GYR_OFFSET_Y_MSB);
        byte GYR_OFFSET_Z_LSB_2 = imu2.read8(GYR_OFFSET_Z_LSB);
        byte GYR_OFFSET_Z_MSB_2 = imu2.read8(GYR_OFFSET_Z_MSB);

        byte ACC_RADIUS_LSB_2 = imu2.read8(ACC_RADIUS_LSB);
        byte ACC_RADIUS_MSB_2 = imu2.read8(ACC_RADIUS_MSB);
        byte MAG_RADIUS_LSB_2 = imu2.read8(MAG_RADIUS_LSB);
        byte MAG_RADIUS_MSB_2 = imu2.read8(MAG_RADIUS_MSB);

        PropertiesLoader propertiesLoader = new PropertiesLoader("IMUCalibration");

        propertiesLoader.setByteProperty("ACC_OFFSET_X_LSB_1", ACC_OFFSET_X_LSB_1);
        propertiesLoader.setByteProperty("ACC_OFFSET_X_MSB_1", ACC_OFFSET_X_MSB_1);
        propertiesLoader.setByteProperty("ACC_OFFSET_Y_LSB_1", ACC_OFFSET_Y_LSB_1);
        propertiesLoader.setByteProperty("ACC_OFFSET_Y_MSB_1", ACC_OFFSET_Y_MSB_1);
        propertiesLoader.setByteProperty("ACC_OFFSET_Z_LSB_1", ACC_OFFSET_Z_LSB_1);
        propertiesLoader.setByteProperty("ACC_OFFSET_Z_MSB_1", ACC_OFFSET_Z_MSB_1);

        propertiesLoader.setByteProperty("MAG_OFFSET_X_LSB_1", MAG_OFFSET_X_LSB_1);
        propertiesLoader.setByteProperty("MAG_OFFSET_X_MSB_1", MAG_OFFSET_X_MSB_1);
        propertiesLoader.setByteProperty("MAG_OFFSET_Y_LSB_1", MAG_OFFSET_Y_LSB_1);
        propertiesLoader.setByteProperty("MAG_OFFSET_Y_MSB_1", MAG_OFFSET_Y_MSB_1);
        propertiesLoader.setByteProperty("MAG_OFFSET_Z_LSB_1", MAG_OFFSET_Z_LSB_1);
        propertiesLoader.setByteProperty("MAG_OFFSET_Z_MSB_1", MAG_OFFSET_Z_MSB_1);

        propertiesLoader.setByteProperty("GYR_OFFSET_X_LSB_1", GYR_OFFSET_X_LSB_1);
        propertiesLoader.setByteProperty("GYR_OFFSET_X_MSB_1", GYR_OFFSET_X_MSB_1);
        propertiesLoader.setByteProperty("GYR_OFFSET_Y_LSB_1", GYR_OFFSET_Y_LSB_1);
        propertiesLoader.setByteProperty("GYR_OFFSET_Y_MSB_1", GYR_OFFSET_Y_MSB_1);
        propertiesLoader.setByteProperty("GYR_OFFSET_Z_LSB_1", GYR_OFFSET_Z_LSB_1);
        propertiesLoader.setByteProperty("GYR_OFFSET_Z_MSB_1", GYR_OFFSET_Z_MSB_1);

        propertiesLoader.setByteProperty("ACC_RADIUS_LSB_1", ACC_RADIUS_LSB_1);
        propertiesLoader.setByteProperty("ACC_RADIUS_MSB_1", ACC_RADIUS_MSB_1);
        propertiesLoader.setByteProperty("MAG_RADIUS_LSB_1", MAG_RADIUS_LSB_1);
        propertiesLoader.setByteProperty("MAG_RADIUS_MSB_1", MAG_RADIUS_MSB_1);

        propertiesLoader.setByteProperty("ACC_OFFSET_X_LSB_2", ACC_OFFSET_X_LSB_2);
        propertiesLoader.setByteProperty("ACC_OFFSET_X_MSB_2", ACC_OFFSET_X_MSB_2);
        propertiesLoader.setByteProperty("ACC_OFFSET_Y_LSB_2", ACC_OFFSET_Y_LSB_2);
        propertiesLoader.setByteProperty("ACC_OFFSET_Y_MSB_2", ACC_OFFSET_Y_MSB_2);
        propertiesLoader.setByteProperty("ACC_OFFSET_Z_LSB_2", ACC_OFFSET_Z_LSB_2);
        propertiesLoader.setByteProperty("ACC_OFFSET_Z_MSB_2", ACC_OFFSET_Z_MSB_2);

        propertiesLoader.setByteProperty("MAG_OFFSET_X_LSB_2", MAG_OFFSET_X_LSB_2);
        propertiesLoader.setByteProperty("MAG_OFFSET_X_MSB_2", MAG_OFFSET_X_MSB_2);
        propertiesLoader.setByteProperty("MAG_OFFSET_Y_LSB_2", MAG_OFFSET_Y_LSB_2);
        propertiesLoader.setByteProperty("MAG_OFFSET_Y_MSB_2", MAG_OFFSET_Y_MSB_2);
        propertiesLoader.setByteProperty("MAG_OFFSET_Z_LSB_2", MAG_OFFSET_Z_LSB_2);
        propertiesLoader.setByteProperty("MAG_OFFSET_Z_MSB_2", MAG_OFFSET_Z_MSB_2);

        propertiesLoader.setByteProperty("GYR_OFFSET_X_LSB_2", GYR_OFFSET_X_LSB_2);
        propertiesLoader.setByteProperty("GYR_OFFSET_X_MSB_2", GYR_OFFSET_X_MSB_2);
        propertiesLoader.setByteProperty("GYR_OFFSET_Y_LSB_2", GYR_OFFSET_Y_LSB_2);
        propertiesLoader.setByteProperty("GYR_OFFSET_Y_MSB_2", GYR_OFFSET_Y_MSB_2);
        propertiesLoader.setByteProperty("GYR_OFFSET_Z_LSB_2", GYR_OFFSET_Z_LSB_2);
        propertiesLoader.setByteProperty("GYR_OFFSET_Z_MSB_2", GYR_OFFSET_Z_MSB_2);

        propertiesLoader.setByteProperty("ACC_RADIUS_LSB_2", ACC_RADIUS_LSB_2);
        propertiesLoader.setByteProperty("ACC_RADIUS_MSB_2", ACC_RADIUS_MSB_2);
        propertiesLoader.setByteProperty("MAG_RADIUS_LSB_2", MAG_RADIUS_LSB_2);
        propertiesLoader.setByteProperty("MAG_RADIUS_MSB_2", MAG_RADIUS_MSB_2);


    }

    /**
     * Loads all of the information from the IMUCalibration.property file and loads them to the imu
     */
    private void loadCalibration(){

        PropertiesLoader propertiesLoader = new PropertiesLoader("IMUCalibration");

        byte ACC_OFFSET_X_LSB_1 = propertiesLoader.getByteProperty("ACC_OFFSET_X_LSB_1");
        byte ACC_OFFSET_X_MSB_1 = propertiesLoader.getByteProperty("ACC_OFFSET_X_MSB_1");
        byte ACC_OFFSET_Y_LSB_1 = propertiesLoader.getByteProperty("ACC_OFFSET_Y_LSB_1");
        byte ACC_OFFSET_Y_MSB_1 = propertiesLoader.getByteProperty("ACC_OFFSET_Y_MSB_1");
        byte ACC_OFFSET_Z_LSB_1 = propertiesLoader.getByteProperty("ACC_OFFSET_Z_LSB_1");
        byte ACC_OFFSET_Z_MSB_1 = propertiesLoader.getByteProperty("ACC_OFFSET_Z_MSB_1");

        byte MAG_OFFSET_X_LSB_1 = propertiesLoader.getByteProperty("MAG_OFFSET_X_LSB_1");
        byte MAG_OFFSET_X_MSB_1 = propertiesLoader.getByteProperty("MAG_OFFSET_X_MSB_1");
        byte MAG_OFFSET_Y_LSB_1 = propertiesLoader.getByteProperty("MAG_OFFSET_Y_LSB_1");
        byte MAG_OFFSET_Y_MSB_1 = propertiesLoader.getByteProperty("MAG_OFFSET_Y_MSB_1");
        byte MAG_OFFSET_Z_LSB_1 = propertiesLoader.getByteProperty("MAG_OFFSET_Z_LSB_1");
        byte MAG_OFFSET_Z_MSB_1 = propertiesLoader.getByteProperty("MAG_OFFSET_Z_MSB_1");

        byte GYR_OFFSET_X_LSB_1 = propertiesLoader.getByteProperty("GYR_OFFSET_X_LSB_1");
        byte GYR_OFFSET_X_MSB_1 = propertiesLoader.getByteProperty("GYR_OFFSET_X_MSB_1");
        byte GYR_OFFSET_Y_LSB_1 = propertiesLoader.getByteProperty("GYR_OFFSET_Y_LSB_1");
        byte GYR_OFFSET_Y_MSB_1 = propertiesLoader.getByteProperty("GYR_OFFSET_Y_MSB_1");
        byte GYR_OFFSET_Z_LSB_1 = propertiesLoader.getByteProperty("GYR_OFFSET_Z_LSB_1");
        byte GYR_OFFSET_Z_MSB_1 = propertiesLoader.getByteProperty("GYR_OFFSET_Z_MSB_1");

        byte ACC_RADIUS_LSB_1 = propertiesLoader.getByteProperty("ACC_RADIUS_LSB_1");
        byte ACC_RADIUS_MSB_1 = propertiesLoader.getByteProperty("ACC_RADIUS_MSB_1");
        byte MAG_RADIUS_LSB_1 = propertiesLoader.getByteProperty("MAG_RADIUS_LSB_1");
        byte MAG_RADIUS_MSB_1 = propertiesLoader.getByteProperty("MAG_RADIUS_MSB_1");

        byte ACC_OFFSET_X_LSB_2 = propertiesLoader.getByteProperty("ACC_OFFSET_X_LSB_2");
        byte ACC_OFFSET_X_MSB_2 = propertiesLoader.getByteProperty("ACC_OFFSET_X_MSB_2");
        byte ACC_OFFSET_Y_LSB_2 = propertiesLoader.getByteProperty("ACC_OFFSET_Y_LSB_2");
        byte ACC_OFFSET_Y_MSB_2 = propertiesLoader.getByteProperty("ACC_OFFSET_Y_MSB_2");
        byte ACC_OFFSET_Z_LSB_2 = propertiesLoader.getByteProperty("ACC_OFFSET_Z_LSB_2");
        byte ACC_OFFSET_Z_MSB_2 = propertiesLoader.getByteProperty("ACC_OFFSET_Z_MSB_2");

        byte MAG_OFFSET_X_LSB_2 = propertiesLoader.getByteProperty("MAG_OFFSET_X_LSB_2");
        byte MAG_OFFSET_X_MSB_2 = propertiesLoader.getByteProperty("MAG_OFFSET_X_MSB_2");
        byte MAG_OFFSET_Y_LSB_2 = propertiesLoader.getByteProperty("MAG_OFFSET_Y_LSB_2");
        byte MAG_OFFSET_Y_MSB_2 = propertiesLoader.getByteProperty("MAG_OFFSET_Y_MSB_2");
        byte MAG_OFFSET_Z_LSB_2 = propertiesLoader.getByteProperty("MAG_OFFSET_Z_LSB_2");
        byte MAG_OFFSET_Z_MSB_2 = propertiesLoader.getByteProperty("MAG_OFFSET_Z_MSB_2");

        byte GYR_OFFSET_X_LSB_2 = propertiesLoader.getByteProperty("GYR_OFFSET_X_LSB_2");
        byte GYR_OFFSET_X_MSB_2 = propertiesLoader.getByteProperty("GYR_OFFSET_X_MSB_2");
        byte GYR_OFFSET_Y_LSB_2 = propertiesLoader.getByteProperty("GYR_OFFSET_Y_LSB_2");
        byte GYR_OFFSET_Y_MSB_2 = propertiesLoader.getByteProperty("GYR_OFFSET_Y_MSB_2");
        byte GYR_OFFSET_Z_LSB_2 = propertiesLoader.getByteProperty("GYR_OFFSET_Z_LSB_2");
        byte GYR_OFFSET_Z_MSB_2 = propertiesLoader.getByteProperty("GYR_OFFSET_Z_MSB_2");

        byte ACC_RADIUS_LSB_2 = propertiesLoader.getByteProperty("ACC_RADIUS_LSB_2");
        byte ACC_RADIUS_MSB_2 = propertiesLoader.getByteProperty("ACC_RADIUS_MSB_2");
        byte MAG_RADIUS_LSB_2 = propertiesLoader.getByteProperty("MAG_RADIUS_LSB_2");
        byte MAG_RADIUS_MSB_2 = propertiesLoader.getByteProperty("MAG_RADIUS_MSB_2");

        imu1.write8(ACC_OFFSET_X_LSB, ACC_OFFSET_X_LSB_1);
        imu1.write8(ACC_OFFSET_X_MSB, ACC_OFFSET_X_MSB_1);
        imu1.write8(ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_LSB_1);
        imu1.write8(ACC_OFFSET_Y_MSB, ACC_OFFSET_Y_MSB_1);
        imu1.write8(ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_LSB_1);
        imu1.write8(ACC_OFFSET_Z_MSB, ACC_OFFSET_Z_MSB_1);

        imu1.write8(MAG_OFFSET_X_LSB, MAG_OFFSET_X_LSB_1);
        imu1.write8(MAG_OFFSET_X_MSB, MAG_OFFSET_X_MSB_1);
        imu1.write8(MAG_OFFSET_Y_LSB, MAG_OFFSET_Y_LSB_1);
        imu1.write8(MAG_OFFSET_Y_MSB, MAG_OFFSET_Y_MSB_1);
        imu1.write8(MAG_OFFSET_Z_LSB, MAG_OFFSET_Z_LSB_1);
        imu1.write8(MAG_OFFSET_Z_MSB, MAG_OFFSET_Z_MSB_1);

        imu1.write8(GYR_OFFSET_X_LSB, GYR_OFFSET_X_LSB_1);
        imu1.write8(GYR_OFFSET_X_MSB, GYR_OFFSET_X_MSB_1);
        imu1.write8(GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_LSB_1);
        imu1.write8(GYR_OFFSET_Y_MSB, GYR_OFFSET_Y_MSB_1);
        imu1.write8(GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_LSB_1);
        imu1.write8(GYR_OFFSET_Z_MSB, GYR_OFFSET_Z_MSB_1);

        imu1.write8(ACC_RADIUS_LSB, ACC_RADIUS_LSB_1);
        imu1.write8(ACC_RADIUS_MSB, ACC_RADIUS_MSB_1);
        imu1.write8(MAG_RADIUS_LSB, MAG_RADIUS_LSB_1);
        imu1.write8(MAG_RADIUS_MSB, MAG_RADIUS_MSB_1);

        imu2.write8(ACC_OFFSET_X_LSB, ACC_OFFSET_X_LSB_2);
        imu2.write8(ACC_OFFSET_X_MSB, ACC_OFFSET_X_MSB_2);
        imu2.write8(ACC_OFFSET_Y_LSB, ACC_OFFSET_Y_LSB_2);
        imu2.write8(ACC_OFFSET_Y_MSB, ACC_OFFSET_Y_MSB_2);
        imu2.write8(ACC_OFFSET_Z_LSB, ACC_OFFSET_Z_LSB_2);
        imu2.write8(ACC_OFFSET_Z_MSB, ACC_OFFSET_Z_MSB_2);

        imu2.write8(MAG_OFFSET_X_LSB, MAG_OFFSET_X_LSB_2);
        imu2.write8(MAG_OFFSET_X_MSB, MAG_OFFSET_X_MSB_2);
        imu2.write8(MAG_OFFSET_Y_LSB, MAG_OFFSET_Y_LSB_2);
        imu2.write8(MAG_OFFSET_Y_MSB, MAG_OFFSET_Y_MSB_2);
        imu2.write8(MAG_OFFSET_Z_LSB, MAG_OFFSET_Z_LSB_2);
        imu2.write8(MAG_OFFSET_Z_MSB, MAG_OFFSET_Z_MSB_2);

        imu2.write8(GYR_OFFSET_X_LSB, GYR_OFFSET_X_LSB_2);
        imu2.write8(GYR_OFFSET_X_MSB, GYR_OFFSET_X_MSB_2);
        imu2.write8(GYR_OFFSET_Y_LSB, GYR_OFFSET_Y_LSB_2);
        imu2.write8(GYR_OFFSET_Y_MSB, GYR_OFFSET_Y_MSB_2);
        imu2.write8(GYR_OFFSET_Z_LSB, GYR_OFFSET_Z_LSB_2);
        imu2.write8(GYR_OFFSET_Z_MSB, GYR_OFFSET_Z_MSB_2);

        imu2.write8(ACC_RADIUS_LSB, ACC_RADIUS_LSB_2);
        imu2.write8(ACC_RADIUS_MSB, ACC_RADIUS_MSB_2);
        imu2.write8(MAG_RADIUS_LSB, MAG_RADIUS_LSB_2);
        imu2.write8(MAG_RADIUS_MSB, MAG_RADIUS_MSB_2);
    }

    Thread imuThread;
    private volatile double currentXAcceleration;
    private volatile double currentYAcceleration;

    private volatile double pastXAcceleration;
    private volatile double pastYAcceleration;

    private volatile double currentXVelocity;
    private volatile double currentYVelocity;

    private volatile double pastXVelocity;
    private volatile double pastYVelocity;

    private volatile double currentXPosition;
    private volatile double currentYPosition;

    private volatile boolean gotFirstAcceleration;
    private volatile boolean gotFirstVelocity;

    int dt = 50;

    void startOfTheThread(){

        pastXAcceleration = 0;
        pastYAcceleration = 0;
        currentXAcceleration = 0;
        currentYAcceleration = 0;

        pastXAcceleration = 0;
        pastYAcceleration = 0;
        currentXAcceleration = 0;
        currentYAcceleration = 0;

        currentXPosition = 0;
        currentYPosition = 0;

        gotFirstAcceleration = false;
        gotFirstVelocity = false;


        imuThread = new Thread(() -> {
            if(!gotFirstAcceleration) {
                pastXAcceleration = currentXAcceleration;
                pastYAcceleration = currentYAcceleration;
            }

            currentXAcceleration = imu1.getLinearAcceleration().xAccel;
            currentYAcceleration = imu1.getLinearAcceleration().yAccel;
            gotFirstAcceleration = true;

            if(gotFirstAcceleration) {
                if(!gotFirstVelocity) {
                    pastXVelocity = currentXVelocity;
                    pastYVelocity = currentYVelocity;
                }

                currentXVelocity = (currentXAcceleration + pastXAcceleration) / 2 * dt;
                currentYVelocity = (currentYAcceleration + pastYAcceleration) / 2 * dt;
                gotFirstVelocity = true;

                if(gotFirstVelocity) {
                    currentXPosition = (currentXVelocity + pastXVelocity) / 2 * dt;
                    currentYPosition = (currentYVelocity + pastYVelocity) / 2 * dt;
                }
            }
            sleep(50);
        });

    }

}