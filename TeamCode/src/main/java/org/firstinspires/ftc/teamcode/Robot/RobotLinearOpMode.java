package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.*;
import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;
import org.firstinspires.ftc.teamcode.IO.RuntimeLogger;

import static java.lang.Math.abs;

//changes this commit
//removed unused thread methods
//changed to use average encoder vals
//testing experimental angle compensation while moving forward/backward

public class RobotLinearOpMode extends Robot {

    private RuntimeLogger logger = new RuntimeLogger("motionProfile");

    private PropertiesLoader propertiesLoader = new PropertiesLoader("Robot");
    private LinearOpMode linearOpMode;
    private double inchesToEncoders = 4000 / 69; //about 60 encoder ticks to an inch
    private double encodersToInches = 69 / 4000; //about 60 encoder ticks to an inch
    private double ROTATION_ACCELERATION_DISTANCE = propertiesLoader.getDoubleProperty("ROTATION_ACCELERATION_DISTANCE");
    private double ROTATION_START_POWER = propertiesLoader.getDoubleProperty("ROTATION_START_POWER");
    private double ROTATION_DECELLERATION_DISTANCE = propertiesLoader.getDoubleProperty("ROTATION_DECELLERATION_DISTANCE");
    private double ROTATION_END_POWER = propertiesLoader.getDoubleProperty("ROTATION_END_POWER");
    private double ROTATION_FULL_POWER = propertiesLoader.getDoubleProperty("ROTATION_FULL_POWER");
    private double FORWARD_ACCELERATION_DISTANCE = propertiesLoader.getDoubleProperty("FORWARD_ACCELERATION_DISTANCE");
    private double FORWARD_START_POWER = propertiesLoader.getDoubleProperty("FORWARD_START_POWER");
    private double FORWARD_DECELLERATION_DISTANCE = propertiesLoader.getDoubleProperty("FORWARD_DECELLERATION_DISTANCE");
    private double FORWARD_END_POWER = propertiesLoader.getDoubleProperty("FORWARD_END_POWER");
    private double FORWARD_FULL_POWER = propertiesLoader.getDoubleProperty("FORWARD_FULL_POWER");
    private double STRAFE_ACCELERATION_DISTANCE = propertiesLoader.getDoubleProperty("STRAFE_ACCELERATION_DISTANCE");
    private double STRAFE_START_POWER = propertiesLoader.getDoubleProperty("STRAFE_START_POWER");
    private double STRAFE_DECELLERATION_DISTANCE = propertiesLoader.getDoubleProperty("STRAFE_DECELLERATION_DISTANCE");
    private double STRAFE_END_POWER = propertiesLoader.getDoubleProperty("STRAFE_END_POWER");
    private double STRAFE_FULL_POWER = propertiesLoader.getDoubleProperty("STRAFE_FULL_POWER");
    private double ARM_LIMIT_ONE = propertiesLoader.getDoubleProperty("ARM_LIMIT_ONE");
    private double ARM_LIMIT_TWO = propertiesLoader.getDoubleProperty("ARM_LIMIT_TWO");
    private double ARM_ACCELERATION_DISTANCE = propertiesLoader.getDoubleProperty("ARM_ACCELERATION_DISTANCE");
    private double ARM_START_POWER = propertiesLoader.getDoubleProperty("ARM_START_POWER");
    private double ARM_DECELLERATION_DISTANCE = propertiesLoader.getDoubleProperty("ARM_DECELLERATION_DISTANCE");
    private double ARM_END_POWER = propertiesLoader.getDoubleProperty("ARM_END_POWER");
    private double ARM_FULL_POWER = propertiesLoader.getDoubleProperty("ARM_FULL_POWER");
    private double LIFT_LIMIT_ONE = propertiesLoader.getDoubleProperty("LIFT_LIMIT_ONE");
    private double LIFT_LIMIT_TWO = propertiesLoader.getDoubleProperty("LIFT_LIMIT_TWO");
    private double LIFT_ACCELERATION_DISTANCE = propertiesLoader.getDoubleProperty("LIFT_ACCELERATION_DISTANCE");
    private double LIFT_START_POWER = propertiesLoader.getDoubleProperty("LIFT_START_POWER");
    private double LIFT_DECELLERATION_DISTANCE = propertiesLoader.getDoubleProperty("LIFT_DECELLERATION_DISTANCE");
    private double LIFT_END_POWER = propertiesLoader.getDoubleProperty("LIFT_END_POWER");
    private double LIFT_FULL_POWER = propertiesLoader.getDoubleProperty("LIFT_FULL_POWER");
    /**
     * SIDE GRABBER POSITIONS
     */
    private double LEFT_STOWED_GRABBER = propertiesLoader.getDoubleProperty("LEFT_STOWED_GRABBER");
    private double LEFT_STOWED_SERVO = propertiesLoader.getDoubleProperty("LEFT_STOWED_SERVO");
    private double LEFT_GRABBING_GRABBER = propertiesLoader.getDoubleProperty("LEFT_GRABBING_GRABBER");
    private double LEFT_GRABBING_SERVO = propertiesLoader.getDoubleProperty("LEFT_GRABBING_SERVO");
    private double LEFT_READY_GRABBER = propertiesLoader.getDoubleProperty("LEFT_READY_GRABBER");
    private double LEFT_READY_SERVO = propertiesLoader.getDoubleProperty("LEFT_READY_SERVO");
    private double LEFT_DEFAULT_GRABBER = propertiesLoader.getDoubleProperty("LEFT_DEFAULT_GRABBER");
    private double LEFT_DEFAULT_SERVO = propertiesLoader.getDoubleProperty("LEFT_DEFAULT_SERVO");
    private double LEFT_GRABBING_SOON_GRABBER = propertiesLoader.getDoubleProperty("LEFT_GRABBING_SOON_GRABBER");
    private double LEFT_GRABBING_SOON_SERVO = propertiesLoader.getDoubleProperty("LEFT_GRABBING_SOON_SERVO");

    private double LEFT_DROPPING_SOON_GRABBER = propertiesLoader.getDoubleProperty("LEFT_DROPPING_SOON_GRABBER");
    private double LEFT_DROPPING_SOON_SERVO = propertiesLoader.getDoubleProperty("LEFT_DROPPING_SOON_SERVO");
    private double LEFT_DROPPING_GRABBER = propertiesLoader.getDoubleProperty("LEFT_DROPPING_GRABBER");
    private double LEFT_DROPPING_SERVO = propertiesLoader.getDoubleProperty("LEFT_DROPPING_SERVO");

    private double RIGHT_STOWED_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_STOWED_GRABBER");
    private double RIGHT_STOWED_SERVO = propertiesLoader.getDoubleProperty("RIGHT_STOWED_SERVO");
    private double RIGHT_GRABBING_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_GRABBING_GRABBER");
    private double RIGHT_GRABBING_SERVO = propertiesLoader.getDoubleProperty("RIGHT_GRABBING_SERVO");
    private double RIGHT_READY_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_READY_GRABBER");
    private double RIGHT_READY_SERVO = propertiesLoader.getDoubleProperty("RIGHT_READY_SERVO");
    private double RIGHT_DEFAULT_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_DEFAULT_GRABBER");
    private double RIGHT_DEFAULT_SERVO = propertiesLoader.getDoubleProperty("RIGHT_DEFAULT_SERVO");
    private double RIGHT_GRABBING_SOON_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_GRABBING_SOON_GRABBER");
    private double RIGHT_GRABBING_SOON_SERVO = propertiesLoader.getDoubleProperty("RIGHT_GRABBING_SOON_SERVO");

    private double RIGHT_DROPPING_SOON_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_DROPPING_SOON_GRABBER");
    private double RIGHT_DROPPING_SOON_SERVO = propertiesLoader.getDoubleProperty("RIGHT_DROPPING_SOON_SERVO");
    private double RIGHT_DROPPING_GRABBER = propertiesLoader.getDoubleProperty("RIGHT_DROPPING_GRABBER");
    private double RIGHT_DROPPING_SERVO = propertiesLoader.getDoubleProperty("RIGHT_DROPPING_SERVO");

    private BalisticProfile fastRotationBalisticProfile = new BalisticProfile(ROTATION_ACCELERATION_DISTANCE, ROTATION_DECELLERATION_DISTANCE, ROTATION_START_POWER, ROTATION_FULL_POWER, ROTATION_END_POWER, BalisticProfile.CURVE_TYPE.SINUSOIDAL_SCURVE, BalisticProfile.CURVE_TYPE.LINEAR);
    private BalisticProfile forwardBalisticProfile = new BalisticProfile(FORWARD_ACCELERATION_DISTANCE, FORWARD_DECELLERATION_DISTANCE, FORWARD_START_POWER, FORWARD_FULL_POWER, FORWARD_END_POWER, BalisticProfile.CURVE_TYPE.LINEAR, BalisticProfile.CURVE_TYPE.LINEAR);
    private BalisticProfile strafeBalisticProfile = new BalisticProfile(STRAFE_ACCELERATION_DISTANCE, STRAFE_DECELLERATION_DISTANCE, STRAFE_START_POWER, STRAFE_FULL_POWER, STRAFE_END_POWER, BalisticProfile.CURVE_TYPE.LINEAR, BalisticProfile.CURVE_TYPE.LINEAR);
    private BalisticProfile armBalisticProfile = new BalisticProfile(ARM_ACCELERATION_DISTANCE, ARM_DECELLERATION_DISTANCE, ARM_START_POWER, ARM_FULL_POWER, ARM_END_POWER, ARM_LIMIT_ONE, ARM_LIMIT_TWO, BalisticProfile.CURVE_TYPE.LINEAR, BalisticProfile.CURVE_TYPE.LINEAR);
    //private BalisticProfile liftBetterBalisticProfile = new BalisticProfile(LIFT_ACCELERATION_DISTANCE, LIFT_DECELLERATION_DISTANCE, LIFT_START_POWER, LIFT_FULL_POWER, LIFT_END_POWER, LIFT_LIMIT_ONE, LIFT_LIMIT_TWO, LINEAR, LINEAR);

    /**
     * CONSTRUCTORS
     */

    public RobotLinearOpMode(LinearOpMode linearOpMode) {

        super(linearOpMode);
        this.linearOpMode = linearOpMode;
        init();
    }

    public double getInchesToEncoders() {
        return inchesToEncoders;
    }

    private void init() {
        linearOpMode.telemetry.addData("Status", " DO NOT START");
        linearOpMode.telemetry.update();

        setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);

        initIMU();
        calibration();

        holdMarker();
        openGrabber();

        setLeftGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.DEFAULT);
        setRightGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.DEFAULT);

        linearOpMode.telemetry.addData("Status", "Initialized");
        linearOpMode.telemetry.update();
    }

    /**
     * DRIVE MOTOR METHODS
     */

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

    public void mecanumPowerDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe - rotation);
    }

    public void compensatedMecanumPowerDrive(
            double strafe,
            double forward,
            double rotation,
            double ratio
    ) {
        ratio = Math.max(abs(ratio), 1) * Math.signum(ratio);

        if (strafe != 0) {
            ratio = Math.max(abs(ratio) / 2, 1) * Math.signum(ratio);
        }

        if ((ratio < 0 && forward > 0) || (ratio > 0 && forward < 0)) {
            leftFrontDriveMotor.setPower((forward - strafe + rotation) / abs(ratio));
            leftBackDriveMotor.setPower((forward + strafe + rotation) / abs(ratio));
            rightFrontDriveMotor.setPower(forward + strafe - rotation);
            rightBackDriveMotor.setPower(forward - strafe - rotation);
        } else if ((ratio > 0 && forward > 0) || (ratio < 0 && forward < 0)) {
            leftFrontDriveMotor.setPower((forward - strafe + rotation));
            leftBackDriveMotor.setPower((forward + strafe + rotation));
            rightFrontDriveMotor.setPower((forward + strafe - rotation) / abs(ratio));
            rightBackDriveMotor.setPower((forward - strafe - rotation) / abs(ratio));
        } else if ((ratio > 0 && strafe > 0) || (ratio < 0 && strafe < 0)) {
            leftFrontDriveMotor.setPower((forward - strafe + rotation) / abs(ratio));
            leftBackDriveMotor.setPower(forward + strafe + rotation);
            rightFrontDriveMotor.setPower((forward + strafe - rotation) / abs(ratio));
            rightBackDriveMotor.setPower(forward - strafe - rotation);
        } else if ((ratio < 0 && strafe > 0) || (ratio > 0 && strafe < 0)) {
            leftFrontDriveMotor.setPower(forward - strafe + rotation);
            leftBackDriveMotor.setPower((forward + strafe + rotation) / abs(ratio));
            rightFrontDriveMotor.setPower(forward + strafe - rotation);
            rightBackDriveMotor.setPower((forward - strafe - rotation) / abs(ratio));
        }

    }

//    void compensatedMecanumDrive(MOVEMENT_DIRECTION movement_direction,
//                            double motorpower,
//                            double currentAngle,
//                                 double desiredAngle){
//
//        IMUControl imuControl = new IMUControl();
//        double[] driveControls = new double[3];
//
//
//        switch (movement_direction) {
//            case FORWARD:
//                driveControls[1] = motorpower;
//
//                break;
//            case STRAFE:
//                driveControls[0] = motorpower;
//                break;
//        }
//
//        imuControl.compensate(driveControls, -angleOff);
//        drive(driveControls[0], driveControls[1], angleOff);
//
//
//
//    }

    public void stopDriveMotors() {
        mecanumPowerDrive(0, 0, 0);
    }

    public void turnToDegreeFast(double desiredEndRotation) {

        double startRotation = getRev10IMUAngle()[2];
        double currentRotation;
        double motorPower;

        if (abs(desiredEndRotation - startRotation + 360) < abs(desiredEndRotation - startRotation)) {
            desiredEndRotation += 360;
        } else if (abs(desiredEndRotation - startRotation - 360) < abs(desiredEndRotation - startRotation)) {
            desiredEndRotation -= 360;
        }

        fastRotationBalisticProfile.setCurve(startRotation, desiredEndRotation);

        while (!fastRotationBalisticProfile.isDone() && linearOpMode.opModeIsActive()) {
            currentRotation = getRev10IMUAngle()[2];

            if (desiredEndRotation < 0 && currentRotation > 0) {
                currentRotation -= 360;
            } else if (desiredEndRotation > 0 && currentRotation < 0) {
                currentRotation += 360;
            }

            fastRotationBalisticProfile.setCurrentPosition(currentRotation);

            motorPower = fastRotationBalisticProfile.getCurrentPowerAccelDecel();
            mecanumPowerDrive(MOVEMENT_DIRECTION.ROTATION, motorPower);

            //linearOpMode.telemetry.addLine("Motor Power:" + motorPower);
            //linearOpMode.telemetry.update();

            linearOpMode.telemetry.addData("Rotation", currentRotation);
            linearOpMode.telemetry.addData("Power", motorPower);
            linearOpMode.telemetry.update();
        }

        if (fastRotationBalisticProfile.getEnd_power() == 0) {
            stopDriveMotors();
        }

    }

    private double calcRatio(double desiredAngle) {
        double currentAngle = getRev10IMUAngle()[2];
        double testDif = currentAngle - desiredAngle;
        double ratio;

        while (testDif > 180) {
            testDif -= 360;
        }
        ;

        while (testDif < -180) {
            testDif += 360;
        }
        ;

        if (testDif < -90) {
            ratio = 3;
        } else if (testDif > 90) {
            ratio = -3;
        } else {
            ratio = abs(testDif / 30) + 1;

            if (testDif > 0) {
                ratio *= -1;
            }

        }

        linearOpMode.telemetry.addData("Ratio", ratio);
        linearOpMode.telemetry.addData("Difference", testDif);
        linearOpMode.telemetry.update();
        return ratio;
    }

    /**
     * Steers robot ever so slightly during movement to account for rotation differences
     */
    public void compensatingMoveByInchesFast(
            double desiredPositionChangeInInches,
            MOVEMENT_DIRECTION movement_direction,
            double desiredAngle
    ) {
        double startPosition = getAverageDriveTrainEncoder(movement_direction);
        double endPosition = startPosition + desiredPositionChangeInInches * inchesToEncoders;
        double currentPosition;
        double motorPower;

        BalisticProfile balisticProfile;

        switch (movement_direction) {
            case STRAFE:
                balisticProfile = strafeBalisticProfile;
                break;
            default:
                balisticProfile = forwardBalisticProfile;
                break;
        }

        balisticProfile.setCurve(startPosition, endPosition);

        while (!balisticProfile.isDone() && linearOpMode.opModeIsActive()) {
            currentPosition = getAverageDriveTrainEncoder(movement_direction);
            balisticProfile.setCurrentPosition(currentPosition);
            motorPower = balisticProfile.getCurrentPowerAccelDecel();

            if (movement_direction == MOVEMENT_DIRECTION.STRAFE) {
                compensatedMecanumPowerDrive(motorPower, 0, 0, calcRatio(desiredAngle) * balisticProfile.getPercentLeft());
            } else if (movement_direction == MOVEMENT_DIRECTION.FORWARD) {
                compensatedMecanumPowerDrive(0, motorPower, 0, calcRatio(desiredAngle) * balisticProfile.getPercentLeft());
            } else {
                mecanumPowerDrive(movement_direction, motorPower);
            }

            logger.write((motorPower * calcRatio(desiredAngle) * balisticProfile.getPercentLeft()) + " " + (currentPosition - startPosition) + "\n");
        }

        if (balisticProfile.getEnd_power() == 0) {
            stopDriveMotors();
        }

        logger.write("\n");
    }

    public void moveByInchesFast(
            double desiredPositionChangeInInches,
            MOVEMENT_DIRECTION movement_direction
    ) {
        double startPosition = getAverageDriveTrainEncoder(movement_direction);
        double endPosition = startPosition + desiredPositionChangeInInches * inchesToEncoders;
        double currentPosition;
        double motorPower;

        BalisticProfile balisticProfile;

        switch (movement_direction) {
            case STRAFE:
                balisticProfile = strafeBalisticProfile;
                break;
            case FORWARD:
            default:
                balisticProfile = forwardBalisticProfile;
        }

        balisticProfile.setCurve(startPosition, endPosition);

        while (!balisticProfile.isDone() && linearOpMode.opModeIsActive()) {

            currentPosition = getAverageDriveTrainEncoder(movement_direction);
            balisticProfile.setCurrentPosition(currentPosition);

            motorPower = balisticProfile.getCurrentPowerAccelDecel();
            mecanumPowerDrive(movement_direction, motorPower);

            linearOpMode.telemetry.addData("Encoder", currentPosition);
            linearOpMode.telemetry.update();
        }

        if (balisticProfile.getEnd_power() == 0) {
            stopDriveMotors();
        }


    }

    /**
     * INTAKE+LIFT MOTOR METHODS
     */
    public void setIntakePower(double intakePower) {

        if (getIntakeSensorNotPressed()) {
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
        double currentEncoderValue;
        double adjustedMotorPower;

        armBalisticProfile.setCurve(startEncoderValue, endEncoderValue);

        do {
            currentEncoderValue = armMotor.getCurrentPosition();
            armBalisticProfile.setCurrentPosition(currentEncoderValue);


            adjustedMotorPower = armBalisticProfile.getCurrentPowerAccelDecel();
            setArmPower(adjustedMotorPower);

            linearOpMode.telemetry.addData("Arm Encoder", currentEncoderValue);
            linearOpMode.telemetry.update();

        }
        while ((abs(desiredPositionChangeInEncoders) > abs(startEncoderValue - currentEncoderValue)) && linearOpMode.opModeIsActive());

        setArmPower(0);
    }

    /**
     * SERVO METHODS
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
        markerLatchServo.setPosition(0.3);
    }

    //LATCH IS LEFT/RIGHT PRETENDING LATCH IS FRONT OF ROBOT
    public void setLatchPosition(double pos) {
        leftLatchServo.setPosition(pos);
        rightLatchServo.setPosition(pos + .05);
    }

    public void setLeftGrabberPosition(double grabberPos, double servoPos) {
        leftSideGrabber.setPosition(grabberPos);
        leftSideGrabberServo.setPosition(servoPos);
    }

    public void setLeftGrabberPosition(GRABBER_POSITION pos) {

        switch (pos) {
            case DEFAULT:
                leftSideGrabber.setPosition(LEFT_DEFAULT_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_DEFAULT_SERVO);
                break;
            case READY:
                leftSideGrabber.setPosition(LEFT_READY_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_READY_SERVO);
                break;
            case GRABBING:
                leftSideGrabber.setPosition(LEFT_GRABBING_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_GRABBING_SERVO);
                break;
            case STOWED:
                leftSideGrabber.setPosition(LEFT_STOWED_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_STOWED_SERVO);
                break;
            case GRABBING_SOON:
                leftSideGrabber.setPosition(LEFT_GRABBING_SOON_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_GRABBING_SOON_SERVO);
                break;
            case DROPPING_SOON:
                leftSideGrabber.setPosition(LEFT_DROPPING_SOON_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_DROPPING_SOON_SERVO);
                break;
            case DROPPING:
                leftSideGrabber.setPosition(LEFT_DROPPING_GRABBER);
                leftSideGrabberServo.setPosition(LEFT_DROPPING_SERVO);
                break;
        }

    }

    public void setRightGrabberPosition(GRABBER_POSITION pos) {

        switch (pos) {
            case DEFAULT:
                rightSideGrabber.setPosition(RIGHT_DEFAULT_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_DEFAULT_SERVO);
                break;
            case READY:
                rightSideGrabber.setPosition(RIGHT_READY_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_READY_SERVO);
                break;
            case GRABBING:
                rightSideGrabber.setPosition(RIGHT_GRABBING_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_GRABBING_SERVO);
                break;
            case STOWED:
                rightSideGrabber.setPosition(RIGHT_STOWED_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_STOWED_SERVO);
                break;
            case GRABBING_SOON:
                rightSideGrabber.setPosition(RIGHT_GRABBING_SOON_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_GRABBING_SOON_SERVO);
                break;
            case DROPPING_SOON:
                rightSideGrabber.setPosition(RIGHT_DROPPING_SOON_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_DROPPING_SOON_SERVO);
                break;
            case DROPPING:
                rightSideGrabber.setPosition(RIGHT_DROPPING_GRABBER);
                rightSideGrabberServo.setPosition(RIGHT_DROPPING_SERVO);
                break;
        }

    }

    public void setRightGrabberPosition(double grabberPos, double servoPos) {
        rightSideGrabber.setPosition(grabberPos);
        rightSideGrabberServo.setPosition(servoPos);
    }

    /**
     * ENCODER MEASUREMENT METHODS
     */
    public double getAverageDriveTrainEncoder(MOVEMENT_DIRECTION movement_direction) {
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

    private double[] getChangeInDriveTrainEncoder() {
        return new double[]{
                //(leftBackDriveMotor.getCurrentPosition()),
                //(leftBackDriveMotor.getCurrentPosition()),
                //(leftBackDriveMotor.getCurrentPosition()),

                (+leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() + rightBackDriveMotor.getCurrentPosition()) / 4,
                (-leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
                (+leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() - rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,

        };
    }

    //    public double[] getChangeInDriveTrainEncoder() {
    //        return new double[]{
    //        (+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() + rightBackDriveMotor.getCurrentPosition()) / 4,
    //        (- leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() + rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
    //        (+ leftFrontDriveMotor.getCurrentPosition() + leftBackDriveMotor.getCurrentPosition() - rightFrontDriveMotor.getCurrentPosition() - rightBackDriveMotor.getCurrentPosition()) / 4,
    //        };
    //    }

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

    private void calibration() {

        while (!IMUSAreCalibrated() && linearOpMode.opModeIsActive()) {

        }

    }

    public enum MOVEMENT_DIRECTION {
        STRAFE,
        FORWARD,
        ROTATION,
    }

    public enum GRABBER_POSITION {
        DEFAULT,
        READY,
        GRABBING,
        STOWED,
        GRABBING_SOON,
        DROPPING_SOON,
        DROPPING
    }

}