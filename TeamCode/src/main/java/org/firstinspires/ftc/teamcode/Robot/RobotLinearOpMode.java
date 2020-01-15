package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
import org.firstinspires.ftc.teamcode.IO.RuntimeLogger;

import static java.lang.Math.abs;

public class RobotLinearOpMode extends Robot {

    private LinearOpMode linearOpMode;

    private double initialArmPosition = 0; //Initial encoder position of arm at resting position
    private double topArmEncoder = 2300;    //Upper limit of arm encoder
    private double bottomArmEncoder = 0;    //Lower limit of arm encoder
    private double inchesToEncoders = 4000 / 69; //about 60 encoder ticks to an inch

    private RuntimeLogger logger = new RuntimeLogger("MotionProfile");

    /**
     * MOTION PROFILES
     */
    public BallisticMotionProfile armProfile = new BallisticMotionProfile(topArmEncoder, bottomArmEncoder, 1000, 0.2, 1, .6);
    public BallisticMotionProfile liftProfile = new BallisticMotionProfile(
            20,
            1300,
            100,
            .05,
            1,
            .8
    );


    void init() {
        setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        initIMU();
        calibration();

        linearOpMode.telemetry.addData("Status", "Initialized");
        linearOpMode.telemetry.update();
    }


    /**  CONSTRUCTORS  */

    public RobotLinearOpMode(LinearOpMode linearOpMode, COLOR_SENSOR activatedSensor) {
        super(linearOpMode, activatedSensor);
        this.linearOpMode = linearOpMode;
        init();
    }


    public RobotLinearOpMode(LinearOpMode linearOpMode) {

        super(linearOpMode, COLOR_SENSOR.UNDER);
        this.linearOpMode = linearOpMode;
        init();
    }




    /**  DRIVE MOTOR METHODS  */

    private void mecanumPowerDrive(MOVEMENT_DIRECTION movement_direction, double power) {
        switch (movement_direction) {
            case STRAFE:
                mecanumPowerDrive(power, 0, 0);
                break;
            case FORWARD:
                mecanumPowerDrive(0, power, 0);
                break;
            case ROTATION:
                mecanumPowerDrive(0, 0, power);
                break;
        }
    }

    public void mecanumPowerDrive(double[] controller) {
        mecanumPowerDrive(controller[0], controller[1], controller[2]);
    }

    public void mecanumPowerDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe - rotation);
    }

    //compensates for change in center of gravity with arm swinging behind the robot
    public void compensatedMecanumPowerDrive(double strafe, double forward, double rotation, double ratio) {
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe * ratio + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe * ratio - rotation);
    }

    /**
     * Turns relative to starting position
     * i.e. starts at 0, 90 will always mean the same position, moving counter-clockwise
     */

    public void turnToDegree(double endRotation) {

        double currentRotation;
        double adjustedMotorPower;
        double startRotation;

        BallisticMotionProfile TurnProfile = new BallisticMotionProfile(0, 0, 90, .1, 1, 0.5);

        startRotation = getRev10IMUAngle()[2];

        do {

            currentRotation = getRev10IMUAngle()[2];

            linearOpMode.telemetry.addData("Heading: ", currentRotation);
            linearOpMode.telemetry.update();

            adjustedMotorPower = TurnProfile.RunToPositionWithoutAccel(startRotation, currentRotation, endRotation);

            mecanumPowerDrive(MOVEMENT_DIRECTION.ROTATION, adjustedMotorPower);


        } while (((abs(endRotation - currentRotation) > 2) ||
                (abs(getRev10IMUAngularVelocity()[2]) < 10 )) &&
                linearOpMode.opModeIsActive());

        //stopAllMotors();
        stopDriveMotors();
    }

    private void stopDriveMotors() {
        mecanumPowerDrive(0,0,0);
    }


    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction) {
        moveByInches(desiredPositionChangeInInches, movement_direction, .05, 0.8);
    }

    public void moveByInchesMinPower(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double minPower) {
        moveByInches(desiredPositionChangeInInches, movement_direction, minPower, .8);

    }
    public void moveByInchesMaxPower(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double maxPower) {
        moveByInches(desiredPositionChangeInInches, movement_direction, .05, maxPower);
    }

    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, double minPower, double maxPower) {
        double currentAverageEncoderValue;
        double adjustedMotorPower;
        double startDriveTrainEncoders;

        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 400, minPower, 1, maxPower);

        double desiredPositionChangeInEncoders = desiredPositionChangeInInches * inchesToEncoders;

        setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startDriveTrainEncoders = getAverageDriveTrainEncoder(movement_direction);

        do {
            currentAverageEncoderValue = getAverageDriveTrainEncoder(movement_direction);
            adjustedMotorPower = DriveProfile.RunToPositionWithAccel(startDriveTrainEncoders, currentAverageEncoderValue, desiredPositionChangeInEncoders);
            mecanumPowerDrive(movement_direction, adjustedMotorPower);

        } while ((abs(desiredPositionChangeInEncoders - currentAverageEncoderValue) > 20) && linearOpMode.opModeIsActive());


        stopDriveMotors();
    }

    public enum MOVEMENT_DIRECTION {
        STRAFE,
        FORWARD,
        ROTATION,
    }

    /**
     * INTAKE+LIFT MOTOR METHODS
     */
    public void setIntakePower(double intakePower) {
        if (!getIntakeSensorPressed()) {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(intakePower);
        } else {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(-intakePower);
        }
    }

    public void setLiftPower(double liftPower) {
        liftMotor.setPower(liftPower);
    }

    public void setArmPower(double armPower) {
        armMotor.setPower(armPower);
    }

    public void moveArmByEncoder(double desiredPositionChangeInEncoders) {

        double startEncoderValue = armMotor.getCurrentPosition();
        double endEncoderValue = desiredPositionChangeInEncoders + startEncoderValue;

        double currentEncoderValue = 0;
        double adjustedMotorPower = 0;


        do {
            currentEncoderValue = armMotor.getCurrentPosition();

            adjustedMotorPower = armProfile.RunToPositionWithAccel(startEncoderValue, currentEncoderValue, endEncoderValue);

            setArmPower(adjustedMotorPower);

            linearOpMode.telemetry.addData("Arm Encoder", currentEncoderValue);
            linearOpMode.telemetry.update();

        } while ((abs(desiredPositionChangeInEncoders) > abs(startEncoderValue - currentEncoderValue)) && linearOpMode.opModeIsActive());

        setArmPower(0);
    }

    public void threadedMoveArmByEncoder(double desiredPosition){

        Thread localThread = new Thread(() -> {moveArmByEncoder(desiredPosition);});
        localThread.start();

    }

//    public void deploy(){
//        //Deploy function
//        setLiftPower(-0.2);
//        linearOpMode.sleep(2000);
//
//        setArmPower(.2);
//        linearOpMode.sleep(600);
//        setLiftPower(0.2);
//        linearOpMode.sleep(150);
//        setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
//        setLiftPower(0);
//        setArmPower(-0.2);
//        linearOpMode.sleep(50);
//        setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
//        setArmPower(0);
//        linearOpMode.sleep(1000);
//        setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        setArmRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }

    //Look at this method again, see if necessary
    public boolean armHasArrived(double targetEncoder) {
        boolean arrivedAtTargetEncoder;

        //if the target encoder position is below the very starting one, and the current position is below that
        //now we make a variable to use later which represents the initial position when doing a runtoposition command
        double initialArmPosition = 0;
        if ((initialArmPosition < targetEncoder) && (targetEncoder < getArmEncoder())) {
            arrivedAtTargetEncoder = true;
        }
        //the exact opposite condidion that yields essentially the same result
        else if ((initialArmPosition > targetEncoder) && (targetEncoder > getArmEncoder())) {
            arrivedAtTargetEncoder = true;
        } else {
            arrivedAtTargetEncoder = false;
        }

        return arrivedAtTargetEncoder;
    }

    /**
     * LATCH+CLAW+MARKER METHODS
     */
    public void openGrabber() {
        blockGrabberServo.setPosition(.7);
    }

    public void closeGrabber() {
        blockGrabberServo.setPosition(.4);
    }

    public void dropMarker() {
        markerLatchServo.setPosition(1);
    }

    public void holdMarker() {
        markerLatchServo.setPosition(0);
    }

    //LATCH IS LEFT/RIGHT PRETENDING LATCH IS FRONT OF ROBOT
    public void setLatchPosition(double pos){
        leftLatchServo.setPosition(pos);
        rightLatchServo.setPosition(pos + .05);
    }

    /**
     * GET ENCODER METHODS
     */
    private double getAverageDriveTrainEncoder(MOVEMENT_DIRECTION movement_direction) {
        switch (movement_direction) {
            case FORWARD:
                return getChangeInDriveTrainEncoder()[0];
            case STRAFE:
                return getChangeInDriveTrainEncoder()[1];
            case ROTATION:
                return getChangeInDriveTrainEncoder()[2];
        }

        return 0;
    }

//    public double[] getChangeInDriveTrainEncoder() {
//        return new double[]{
//        (+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() + rightBackDriveMotor.getCurrentPosition()) / 4,
//        (- leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
//        (+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() - rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
//        };
//    }

    public double[] getChangeInDriveTrainEncoder() {
        return new double[]{
                (leftBackDriveMotor.getCurrentPosition()),
                (leftBackDriveMotor.getCurrentPosition()),
                (leftBackDriveMotor.getCurrentPosition()),

        //(+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() + rightBackDriveMotor.getCurrentPosition()) / 4,
        //(- leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
        //(+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() - rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,

        };
    }

    public int getLiftEncoder() {
        return liftMotor.getCurrentPosition();
    }

    public int getArmEncoder() {
        return armMotor.getCurrentPosition();
    }

    /**
     * STOP METHODS
     */
    public void stopAllMotors() {
        leftFrontDriveMotor.setPower(0);
        leftBackDriveMotor.setPower(0);
        rightFrontDriveMotor.setPower(0);
        rightBackDriveMotor.setPower(0);
        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);
        liftMotor.setPower(0);
        armMotor.setPower(0);
    }

    //eliminates residual forces
    public void stopMotorsAndWait(double seconds) {
        mecanumPowerDrive(0, 0, 0);
        linearOpMode.sleep((int) (seconds * 1000));
    }

    public void calibration() {

        while (!IMUSAreCalibrated() && linearOpMode.opModeIsActive()) {
            linearOpMode.telemetry.addData("Status"," IMUs calibrating");
            linearOpMode.telemetry.update();
        }

    }

}