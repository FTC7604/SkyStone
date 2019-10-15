package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;

import static java.lang.Math.abs;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.MOVEMENT_DIRECTION.*;

public class RobotLinearOpMode extends Robot {

    LinearOpMode linearOpMode;

    double encodersToInches = 69/4000;
    double inchesToEncoders = 4000/69;

    //now we make a variable to use later which represents the initial position when doing a runtoposition command
    public double initialArmPosition = 0;

    //constructor
    public RobotLinearOpMode(LinearOpMode linearOpMode) {

        //creates the robot so that I can use all of the motors
        super(linearOpMode);

        //needs the linear opmode so that I can use telemetry and opModeIsActive()
        this.linearOpMode = linearOpMode;
    }

    //sets the powers on either a power or velocity, and with variables or an array, where 0 is strafe, 1 is forward, and 2 is rotation
    //if I control it using an array, then it just sends it into an earlier method
    public void mecanumPowerDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe - rotation);
    }

    public void mecanumPowerDrive(double[] controller) {
        mecanumPowerDrive(controller[0], controller[1], controller[2]);
    }

    public void mecVelocityDrive(double strafe, double forward, double rotation) {
        leftFrontDriveMotor.setVelocity(forward - strafe + rotation);
        leftBackDriveMotor.setVelocity(forward + strafe + rotation);
        rightFrontDriveMotor.setVelocity(forward + strafe - rotation);
        rightBackDriveMotor.setVelocity(forward - strafe - rotation);
    }

    public void mecVelocityDrive(double[] controller) {
        mecVelocityDrive(controller[0], controller[1], controller[2]);
    }

    //outputs the average of the 4 drive train motors, be sure to reset the encoders before you engage this.
    public double getAverageDriveTrainEncoder(MOVEMENT_DIRECTION movement_direction){
        double averageEncoderPosition = 0;

        switch (movement_direction){

            case STRAFE:
                averageEncoderPosition -= leftFrontDriveMotor.getCurrentPosition();
                averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
                averageEncoderPosition += rightFrontDriveMotor.getCurrentPosition();
                averageEncoderPosition -= rightBackDriveMotor.getCurrentPosition();

                return averageEncoderPosition / 4;
            case FORWARD:
                averageEncoderPosition += leftFrontDriveMotor.getCurrentPosition();
                averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
                averageEncoderPosition += rightFrontDriveMotor.getCurrentPosition();
                averageEncoderPosition += rightBackDriveMotor.getCurrentPosition();

                return averageEncoderPosition / 4;
            case ROTATION:

                averageEncoderPosition += leftFrontDriveMotor.getCurrentPosition();
                averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
                averageEncoderPosition -= rightFrontDriveMotor.getCurrentPosition();
                averageEncoderPosition -= rightBackDriveMotor.getCurrentPosition();

                return averageEncoderPosition / 4;

        }

        return 0;
    }
    public double getAverageForwardDriveTrainEncoder() {
        double averageEncoderPosition = 0;

        averageEncoderPosition += leftFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
        averageEncoderPosition += rightFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += rightBackDriveMotor.getCurrentPosition();

        return averageEncoderPosition / 4;
    }

    public double getAverageStrafeDriveTrainEncoder() {
        double averageEncoderPosition = 0;

        averageEncoderPosition -= leftFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
        averageEncoderPosition += rightFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition -= rightBackDriveMotor.getCurrentPosition();

        return averageEncoderPosition / 4;
    }

    public double getAverageRotationDriveTrainEncoder() {
        double averageEncoderPosition = 0;

        averageEncoderPosition += leftFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition += leftBackDriveMotor.getCurrentPosition();
        averageEncoderPosition -= rightFrontDriveMotor.getCurrentPosition();
        averageEncoderPosition -= rightBackDriveMotor.getCurrentPosition();

        return averageEncoderPosition / 4;
    }

    //two methods that turn precisely, which Casey made and I don't fully understand
    public void turnByDegree(double degree) {
        initIMU();

        BallisticMotionProfile TurnProfile = new BallisticMotionProfile(0, 0, 90, .05, 1, .65);

        double currentAngle;
        double adjustedMotorPower;

        //gets the current angle from the IMU
        double startAngle = getRev2IMUAngle()[2];

        //calculates the angles the angle that it needs to be at, at the end
        double neededAngle = startAngle + degree;
        currentAngle = startAngle;

        //the loop that will run until the needed Angle is acheived
        if (currentAngle < neededAngle) {
            while ((currentAngle < neededAngle && linearOpMode.opModeIsActive())) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecanumPowerDrive(0, 0, adjustedMotorPower);
            }
        } else if (currentAngle > neededAngle && linearOpMode.opModeIsActive()) {
            while ((currentAngle > neededAngle)) {//will run until we get there
                currentAngle = getRev2IMUAngle()[2];////IMU something

                adjustedMotorPower = TurnProfile.RunToPositionWithAccel(startAngle, currentAngle, neededAngle);//get a rotation

                mecanumPowerDrive(0, 0, adjustedMotorPower);
            }
        }
    }

    public enum MOVEMENT_DIRECTION {
        STRAFE,
        FORWARD,
        ROTATION,
    }

    public void moveByInches(double desiredPositionChangeInInches, MOVEMENT_DIRECTION movement_direction, boolean withAccel) {

        double currentAverageEncoderValue = 0;
        double desiredPositionChangeInEncoders = 0;
        double adjustedMotorPower = 0;

        BallisticMotionProfile DriveProfile = new BallisticMotionProfile(0, 0, 800, 0.05, 1, 0.75);

        desiredPositionChangeInEncoders = desiredPositionChangeInInches * inchesToEncoders;

        setDriveTrainRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        do {
            if(movement_direction == STRAFE) currentAverageEncoderValue = getAverageStrafeDriveTrainEncoder();
            if(movement_direction == FORWARD) currentAverageEncoderValue = getAverageForwardDriveTrainEncoder();
            if(movement_direction == ROTATION) currentAverageEncoderValue = getAverageRotationDriveTrainEncoder();

            if(withAccel)adjustedMotorPower = DriveProfile.RunToPositionWithAccel(0, currentAverageEncoderValue, desiredPositionChangeInEncoders);
            else adjustedMotorPower = DriveProfile.RunToPositionWithoutAccel(0, currentAverageEncoderValue, desiredPositionChangeInEncoders);

            if(movement_direction == STRAFE) mecanumPowerDrive(adjustedMotorPower, 0, 0);
            if(movement_direction == FORWARD) mecanumPowerDrive(0, adjustedMotorPower, 0);
            if(movement_direction == ROTATION) mecanumPowerDrive(0,0, adjustedMotorPower);

        } while((abs(desiredPositionChangeInEncoders - currentAverageEncoderValue) > 50) && linearOpMode.opModeIsActive());
    }

    //eliminates residual forces
    public void stopMotorsAndWait(double seconds) {
        mecanumPowerDrive(0, 0, 0);
        linearOpMode.sleep((int) (seconds * 1000));
    }

    //sets the intake power and velocity
    public void setIntakePower(double intakePower) {
        if (intakeIsOpen()) {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(-intakePower);
        } else {
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(intakePower);
        }

    }

    public void setIntakeVelocity(double intakeVelocity) {
        if (intakeIsOpen()) {
            rightIntakeMotor.setVelocity(intakeVelocity);
            leftIntakeMotor.setVelocity(-intakeVelocity);
        } else {
            rightIntakeMotor.setVelocity(intakeVelocity);
            leftIntakeMotor.setVelocity(intakeVelocity);
        }
    }

    //sets the lift power and veloctiy
    public void setLiftPower(double liftPower) {
        liftMotor.setPower(liftPower);
    }

    public int getLiftEncoder() {
        return liftMotor.getCurrentPosition();
    }

    //sets the arm power and velocity
    public void setArmPower(double armPower) {
        armMotor.setPower(armPower);
    }

    public int getArmEncoder() {
        return armMotor.getCurrentPosition();
    }

    //tells us if the arm is on targetEncoder
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
}