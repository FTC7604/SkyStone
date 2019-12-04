package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.*;

public class RobotLinearOpMode extends Robot {

    private LinearOpMode linearOpMode;

    //private double encodersToInches = 69/4000;
    private double inchesToEncoders = 4000/69;

    private  double initialArmPosition = 0; //Initial encoder position of arm at resting position
    private double topArmEncoder = 2300;    //Upper limit of arm encoder
    private double bottomArmEncoder = 0;    //Lower limit of arm encoder

    /**  MOTION PROFILES  */
    public BallisticMotionProfile armProfile = new BallisticMotionProfile(topArmEncoder, bottomArmEncoder, 1000, 0.2, 1, .6);
    public BallisticMotionProfile liftProfile = new BallisticMotionProfile(150, 3300, 1000, 0.15, 1, .6);
    public BallisticMotionProfile turnProfile = new BallisticMotionProfile(0, 0, 90, .05, 1, .75);
    public BallisticMotionProfile driveProfile = new BallisticMotionProfile(0, 0, 200, 0.05, 1, 0.8);

    /**  CONSTRUCTOR  */
    public RobotLinearOpMode(LinearOpMode linearOpMode, COLOR_SENSOR activatedSensor) {
        super(linearOpMode, activatedSensor);
        this.linearOpMode = linearOpMode;
    }

    /**  DRIVE MOTOR METHODS  */
    private void mecanumPowerDrive(MOVEMENT_DIRECTION movement_direction, double power) {
        switch (movement_direction){
            case STRAFE: mecanumPowerDrive(power,0,0);
                break;
            case FORWARD: mecanumPowerDrive(0,power,0);
                break;
            case ROTATION: mecanumPowerDrive(0,0,power);
                break;
        }
    }

    public void mecanumPowerDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe - rotation);
    }

    public void mecanumPowerDrive(double[] controller) {
        mecanumPowerDrive(controller[0], controller[1], controller[2]);
    }

    private void mecVelocityDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setVelocity(forward - strafe + rotation);
        leftBackDriveMotor.setVelocity(forward + strafe + rotation);
        rightFrontDriveMotor.setVelocity(forward + strafe - rotation);
        rightBackDriveMotor.setVelocity(forward - strafe - rotation);
    }

    public void mecVelocityDrive(double[] controller) {
        mecVelocityDrive(controller[0], controller[1], controller[2]);
    }

    public void turnByDegree(double degree) {
        double adjustedMotorPower;
        double startAngle = getRev2IMUAngle()[2];
        double neededAngle = startAngle + degree;
        double currentAngle = startAngle;

        if (currentAngle < neededAngle) {

            while ((currentAngle < neededAngle && linearOpMode.opModeIsActive())) {
                currentAngle = getRev2IMUAngle()[2];
                adjustedMotorPower = turnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);
                mecanumPowerDrive(0, 0, adjustedMotorPower);
            }

        } else if (currentAngle > neededAngle && linearOpMode.opModeIsActive()) {

            while ((currentAngle > neededAngle)) {
                currentAngle = getRev2IMUAngle()[2];
                adjustedMotorPower = turnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);
                mecanumPowerDrive(0, 0, adjustedMotorPower);
            }

        }
    }

    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction) {
        double currentAverageEncoderValue;
        double adjustedMotorPower;
        double[] currentEncoderValues;
        double desiredPositionChangeInEncoders = desiredPositionChangeInInches * inchesToEncoders;

        setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        currentEncoderValues = getDriveTrainEncoders(movement_direction);

        do {
            currentAverageEncoderValue = getAverageDriveTrainEncoder(movement_direction, currentEncoderValues);
            adjustedMotorPower = driveProfile.RunToPositionWithAccel(0, currentAverageEncoderValue, desiredPositionChangeInEncoders);
            mecanumPowerDrive(movement_direction, adjustedMotorPower);
        } while((abs(desiredPositionChangeInEncoders - currentAverageEncoderValue) > 50) && linearOpMode.opModeIsActive());

        stopAllMotors();
    }

    public enum MOVEMENT_DIRECTION {
        STRAFE,
        FORWARD,
        ROTATION,
    }

    public void moveArmByEncoder(double desiredPositionChangeInEncoders) {

        double startEncoderValue = getArmEncoder();
        double endEncoderValue = desiredPositionChangeInEncoders + startEncoderValue;

        double currentEncoderValue = 0;
        double adjustedMotorPower = 0;

        do {
            currentEncoderValue = getArmEncoder();

            adjustedMotorPower = armProfile.RunToPositionWithAccel(startEncoderValue, currentEncoderValue, endEncoderValue);

            setArmPower(adjustedMotorPower);

        } while((abs(desiredPositionChangeInEncoders) > abs(startEncoderValue - currentEncoderValue)) && linearOpMode.opModeIsActive());
    }

    public void moveToLatch(double desiredPositionChangeInInches) {

        double currentAverageEncoderValue = 0;
        double desiredPositionChangeInEncoders = 0;
        double adjustedMotorPower = 0;

        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 8 * inchesToEncoders, 0.05, 1, 0.75);

        desiredPositionChangeInEncoders = desiredPositionChangeInInches * inchesToEncoders;

        //setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        openLatch();

        do {
            currentAverageEncoderValue = getAverageDriveTrainEncoder(FORWARD);

            adjustedMotorPower = DriveProfile.RunToPositionWithAccel(0, currentAverageEncoderValue, desiredPositionChangeInEncoders);

            mecanumPowerDrive(0, adjustedMotorPower, 0);

        } while((abs(desiredPositionChangeInEncoders - currentAverageEncoderValue) > 50) && linearOpMode.opModeIsActive());

        closeLatch();
    }

    //Deleted: hasFoundBlock boolean, extraEncoderDistance double
    public void moveToStone(double maxBotPower, double intakePower) {

        double currentAverageEncoderValue = 0;
        double desiredPositionChangeInEncoders = 1000000000;
        double adjustedMotorPower = 0;

        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 800, 0.05, 1, maxBotPower);

        setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        do {
            currentAverageEncoderValue = getAverageDriveTrainEncoder(FORWARD);

            adjustedMotorPower = DriveProfile.RunToPositionWithAccel(0, currentAverageEncoderValue, desiredPositionChangeInEncoders);

            mecanumPowerDrive(0, adjustedMotorPower, 0);

            setIntakePower(intakePower);

        } while(!getBlockSensorPressed() && linearOpMode.opModeIsActive());

    }

    /**  INTAKE+LIFT MOTOR METHODS  */
    public void setIntakePower(double intakePower) {
        if (!intakeIsOpen()) {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(-intakePower);
        } else {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(intakePower);
        }

    }

    public void setIntakeVelocity(double intakeVelocity) {
        if (!intakeIsOpen()) {
            rightIntakeMotor.setVelocity(intakeVelocity);
            leftIntakeMotor.setVelocity(-intakeVelocity);
        } else {
            rightIntakeMotor.setVelocity(intakeVelocity);
            leftIntakeMotor.setVelocity(intakeVelocity);
        }
    }

    public void setLiftPower(double liftPower) {
        liftMotor.setPower(liftPower);
    }

    public void setArmPower(double armPower) {
        armMotor.setPower(armPower);
    }

    public boolean armHasArrived(double targetEncoder) {
        boolean arrivedAtTargetEncoder;

        //if the target encoder position is below the very starting one, and the current position is below that
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

    /**  LATCH+CLAW+MARKER METHODS  */
    public void closeGrabber() {
        blockGrabberServo.setPosition(.7);
    }

    public void openGrabber() {
        blockGrabberServo.setPosition(.4);
    }

    public void dropMarker() {
        markerLatchServo.setPosition(1);
    }

    public void holdMarker() {
        markerLatchServo.setPosition(0);
    }

    public void closeLatch() {
        leftLatchServo.setPosition(.3);
        rightLatchServo.setPosition(.45);
    }

    public void openLatch() {
        leftLatchServo.setPosition(.5);
        rightLatchServo.setPosition(.65);
    }

    /**  GET ENCODER METHODS  */
    private double getAverageDriveTrainEncoder(MOVEMENT_DIRECTION movement_direction){
        return getAverageDriveTrainEncoder(movement_direction,new double[]{0,0,0,0});
    }

    private double getAverageDriveTrainEncoder(MOVEMENT_DIRECTION movement_direction, double[] startEncoderValues){
        double averageEncoderPosition = 0;

        switch (movement_direction){
            case FORWARD:
                averageEncoderPosition += leftFrontDriveMotor.getCurrentPosition() - startEncoderValues[0];
                averageEncoderPosition += leftBackDriveMotor.getCurrentPosition() - startEncoderValues[1];
                averageEncoderPosition += rightFrontDriveMotor.getCurrentPosition() - startEncoderValues[2];
                averageEncoderPosition += rightBackDriveMotor.getCurrentPosition() - startEncoderValues[3];
                break;
            case STRAFE:
                averageEncoderPosition -= leftFrontDriveMotor.getCurrentPosition() - startEncoderValues[0];
                averageEncoderPosition += leftBackDriveMotor.getCurrentPosition() - startEncoderValues[1];
                averageEncoderPosition += rightFrontDriveMotor.getCurrentPosition() - startEncoderValues[2];
                averageEncoderPosition -= rightBackDriveMotor.getCurrentPosition() - startEncoderValues[3];
                break;
            case ROTATION:
                averageEncoderPosition += leftFrontDriveMotor.getCurrentPosition() - startEncoderValues[0];
                averageEncoderPosition += leftBackDriveMotor.getCurrentPosition() - startEncoderValues[1];
                averageEncoderPosition -= rightFrontDriveMotor.getCurrentPosition() - startEncoderValues[2];
                averageEncoderPosition -= rightBackDriveMotor.getCurrentPosition() - startEncoderValues[3];
                break;
        }

        return averageEncoderPosition/4;
    }

    private double [] getDriveTrainEncoders(MOVEMENT_DIRECTION movement_direction){
        switch (movement_direction){
            case FORWARD:
                return new double[]{
                        leftFrontDriveMotor.getCurrentPosition(),
                        leftBackDriveMotor.getCurrentPosition(),
                        rightFrontDriveMotor.getCurrentPosition(),
                        rightBackDriveMotor.getCurrentPosition(),
                };
            case STRAFE:
                return new double[]{
                        -1 * leftFrontDriveMotor.getCurrentPosition(),
                        leftBackDriveMotor.getCurrentPosition(),
                        rightFrontDriveMotor.getCurrentPosition(),
                        -1 * rightBackDriveMotor.getCurrentPosition(),
                };
            case ROTATION:
                return new double[]{
                        leftFrontDriveMotor.getCurrentPosition(),
                        leftBackDriveMotor.getCurrentPosition(),
                        -1 * rightFrontDriveMotor.getCurrentPosition(),
                        -1 * rightBackDriveMotor.getCurrentPosition(),
                };

            default: return new double[]{};
        }
    }

    public int getLiftEncoder() {
        return liftMotor.getCurrentPosition();
    }

    public int getArmEncoder() {
        return armMotor.getCurrentPosition();
    }

    /**  STOP METHODS  */
    public void stopAllMotors(){
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

}