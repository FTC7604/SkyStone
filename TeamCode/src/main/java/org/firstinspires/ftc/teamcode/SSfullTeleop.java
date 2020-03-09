//imports the package so that all the code can be used
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.CaseyMotionProfile;
import org.firstinspires.ftc.teamcode.Control.HumanController;
import org.firstinspires.ftc.teamcode.Control.Toggle;
import org.firstinspires.ftc.teamcode.LED.LedPattern;
import org.firstinspires.ftc.teamcode.LED.LedPatternStep;

@TeleOp(name = "Skystone Main Teleop", group = "Linear Opmode")
public class SSfullTeleop extends LinearOpMode {
    private final double LIFT_HOME_POSITION = -20;
    private final double WEIGHT_COMP_RATIO = 1.15;
    private final double ARM_HOME_POSITION = 0;
    private final double ARM_SCORING_POSITION = 2300;
    private boolean cardPattern = false;
    private boolean rainbowPattern = false;
    //    private ALLIANCE alliance = AutoTransitioner.alliance;
    private RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern dPadPattern;
    private double intakeCorrectionStartTime = -200;
    private double[] driveTrainController = new double[3];
    private CaseyMotionProfile liftProfile = new CaseyMotionProfile(-1300, -10, 500, 0.25, 1, .7);
    private CaseyMotionProfile armProfile = new CaseyMotionProfile(3300, 0, 1000, 0.3, 1, 1);
    private double intakePower = 0;
    private double armPower = 0;
    private double liftPower = 0;
    private double armPosition = 0;
    private double liftPosition = 0;
    private double armHoldPosition = 0;
    private boolean armGoingToScoringPosition = false;
    private boolean armGoingToHomePosition = false;
    private double initialArmPosition = 0;
    private double odometryUp = 1;
    private HumanController humanController = new HumanController(0.1, 1);
    private ElapsedTime runtime = new ElapsedTime();
    LedPattern cardBlinkinPatter = new LedPattern(new LedPatternStep[] {
            new LedPatternStep(RevBlinkinLedDriver.BlinkinPattern.WHITE, .5),
            new LedPatternStep(RevBlinkinLedDriver.BlinkinPattern.RED, .5)}, runtime.time());
    LedPattern rainbowBlinkingPattern = new LedPattern(new LedPatternStep[] {
            new LedPatternStep(RevBlinkinLedDriver.BlinkinPattern.RED, .2),
            new LedPatternStep(RevBlinkinLedDriver.BlinkinPattern.ORANGE, .2),
            new LedPatternStep(RevBlinkinLedDriver.BlinkinPattern.YELLOW, .2),
            new LedPatternStep(RevBlinkinLedDriver.BlinkinPattern.GREEN, .2),
            new LedPatternStep(RevBlinkinLedDriver.BlinkinPattern.BLUE, .2),
            new LedPatternStep(RevBlinkinLedDriver.BlinkinPattern.VIOLET, .2)

    }, runtime.time());
    private DcMotor rightFrontDriveMotor;
    private DcMotor leftFrontDriveMotor;
    private DcMotor rightBackDriveMotor;
    private DcMotor leftBackDriveMotor;
    private DcMotor rightIntakeMotor;
    private DcMotor leftIntakeMotor;
    private DcMotor armMotor;
    private DcMotor liftMotor;
    private Servo leftLatchServo;
    private Servo rightLatchServo;
    private Servo blockGrabberServo;
    private Servo markerLatchServo;
    private Servo odometryRetractServo;
    private DigitalChannel intakeFulla;
    private DigitalChannel intakeFullb;
    private DigitalChannel openIntakeTouchSensor;
    private Toggle latchIsDown = new Toggle(false);
    private Toggle grabberIsEngaged = new Toggle(false);
    private Toggle driveMode = new Toggle(false);
    //private boolean liftGoingToHomePosition = false;//added new
    private Toggle markerDropper = new Toggle(true);

    @Override
    public void runOpMode(){

        InitHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        openGrabber();

        while(opModeIsActive()){

            runDrive();

            runLifter();

            runArm();

            runIntake();

            runServos();

            getPositionTargets();

            sendDriverFeedback();
        }
    }

    //This is the LedPattern method
    private void sendDriverFeedback(){

        if (gamepad1.dpad_down) {
            cardPattern    = true;
            rainbowPattern = false;
        }
        else if (gamepad1.dpad_right) {
            dPadPattern    = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
            cardPattern    = false;
            rainbowPattern = false;
        }
        else if (gamepad1.dpad_up) {
            cardPattern    = false;
            rainbowPattern = true;
        }
        else if (gamepad1.dpad_left) {
            dPadPattern    = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
            cardPattern    = false;
            rainbowPattern = false;
        }
        else if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
            dPadPattern    = null;
            cardPattern    = false;
            rainbowPattern = false;
        }

        if (!intakeFulla.getState() || !intakeFullb.getState()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
            //LEDs flashing bright or something
        }
        else if (latchIsDown.get()) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            //LEDS a certain color to indicate the latch is down
        }
        else {

            //LEDs some other way, maybe an alliance color if we want
            //            if (dPadPattern != null) {
            //                if (!cardPattern && !rainbowPattern) {
            //                    blinkin.setPattern(dPadPattern);
            //                }
            //                else if (cardPattern) {
            //                    cardBlinkinPatter.update(runtime.time());
            //                    blinkin.setPattern(cardBlinkinPatter.getPattern());
            //                }
            //                else {
            //                    rainbowBlinkingPattern.update(runtime.time());
            //                    blinkin.setPattern(rainbowBlinkingPattern.getPattern());
            //                }
            //            }
            //            else if (alliance == BLUE) {
            //                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            //            }
            //            else if (alliance == ALLIANCE.RED) {
            //                blinkin.setPattern(BlinkinPattern.COLOR_WAVES_LAVA_PALETTE);
            //            }
            //            else {
            //                blinkin.setPattern(BlinkinPattern.RAINBOW_WITH_GLITTER);
            //            }
        }

        //Well get rid of this telemetry for lag reduction
        //        telemetry.addData("Arm Position: ", armPosition);
        //        telemetry.addData("Lift Position: ", liftPosition);
        //        telemetry.update();
    }

    //tells us if the arm is on target
    private boolean armHasArrived(double current, double target){
        boolean arrived;

        //make sure we made it depending on which way we came
        if ((initialArmPosition <= target) && (target <= current)) {
            arrived = true;
        }
        else arrived = (initialArmPosition >= target) && (target >= current);
        return arrived;
    }
    //private double initialLiftPosition = 0;

    private void InitHardware(){
        //assigns it to the config
        rightFrontDriveMotor  = hardwareMap.get(DcMotor.class, "lf");
        leftFrontDriveMotor   = hardwareMap.get(DcMotor.class, "rf");
        rightBackDriveMotor   = hardwareMap.get(DcMotor.class, "lb");
        leftBackDriveMotor    = hardwareMap.get(DcMotor.class, "rb");
        rightIntakeMotor      = hardwareMap.get(DcMotor.class, "ri");
        leftIntakeMotor       = hardwareMap.get(DcMotor.class, "li");
        armMotor              = hardwareMap.get(DcMotor.class, "ax");
        liftMotor             = hardwareMap.get(DcMotor.class, "lx");
        leftLatchServo        = hardwareMap.get(Servo.class, "ll");
        rightLatchServo       = hardwareMap.get(Servo.class, "rl");
        blockGrabberServo     = hardwareMap.get(Servo.class, "bg");
        markerLatchServo      = hardwareMap.get(Servo.class, "ml");
        intakeFulla           = hardwareMap.get(DigitalChannel.class, "blt");
        intakeFullb           = hardwareMap.get(DigitalChannel.class, "brt");
        openIntakeTouchSensor = hardwareMap.get(DigitalChannel.class, "it");
        blinkin               = hardwareMap.get(RevBlinkinLedDriver.class, "bk");
        odometryRetractServo  = hardwareMap.get(Servo.class, "or");

        //sets the direction
        rightFrontDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        rightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        leftIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftLatchServo.setDirection(Servo.Direction.FORWARD);
        rightLatchServo.setDirection(Servo.Direction.REVERSE);
        intakeFulla.setMode(DigitalChannel.Mode.INPUT);
        intakeFullb.setMode(DigitalChannel.Mode.INPUT);
        openIntakeTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        //Brake
        rightFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFrontDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoders
        rightFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void closeGrabber(){
        if (!grabberIsEngaged.get()) grabberIsEngaged.update(true);
        blockGrabberServo.setPosition(0);
    }

    private void openGrabber(){
        if (grabberIsEngaged.get()) grabberIsEngaged.update(true);
        blockGrabberServo.setPosition(.7);
    }

    private void dropMarker(){
        markerLatchServo.setPosition(1);

    }

    private void holdMarker(){
        markerLatchServo.setPosition(.3);
    }

    private void closeLatch(){
        leftLatchServo.setPosition(.3);
        rightLatchServo.setPosition(.45);
    }

    private void openLatch(){
        leftLatchServo.setPosition(.8);
        rightLatchServo.setPosition(.95);
    }

    private void mecanumPowerDrive(double strafe, double forward, double rotation){
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe - rotation);

        //        telemetry.addData("leftFrontDriveMotor power", forward - strafe + rotation);
        //        telemetry.addData("leftBackDriveMotor power", forward + strafe + rotation);
        //        telemetry.addData("rightFrontDriveMotor power", forward + strafe - rotation);
        //        telemetry.addData("rightBackDriveMotor power", forward - strafe - rotation);
        //
        //
        //        telemetry.update();
    }

    //compensates for change in center of gravity with arm swinging behind the robot
    private void compensatedMecanumPowerDrive(
            double strafe, double forward, double rotation, double ratio
    ){
        leftFrontDriveMotor.setPower(forward - strafe + rotation);
        leftBackDriveMotor.setPower(forward + strafe * ratio + rotation);
        rightFrontDriveMotor.setPower(forward + strafe - rotation);
        rightBackDriveMotor.setPower(forward - strafe * ratio - rotation);
    }

    private void mecanumPowerDrive(double[] controller){
        mecanumPowerDrive(controller[0], controller[1], controller[2]);
    }

    private void setIntakePower(double intakePower){
        if (!openIntakeTouchSensor.getState()) {//jammed
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(1);
            intakeCorrectionStartTime = runtime.milliseconds();
        }
        else if (runtime.milliseconds() < intakeCorrectionStartTime + 100) {//unjammed but still needs fixing
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(1);
        }
        else {//all clear
            rightIntakeMotor.setPower(intakePower);
            leftIntakeMotor.setPower(intakePower);
        }

    }

    private void runDrive(){
        //sets up the condidtion for the drivetrain
        driveMode.update(gamepad1.right_bumper);

        if (driveMode.get()) {//tank
            driveTrainController[1] = humanController.linearDriveProfile(((-gamepad1.right_stick_y) * (Math.abs(
                    -gamepad1.right_stick_y)) + ((-gamepad1.left_stick_y) * (Math.abs(-gamepad1.left_stick_y)))) / 2);
            driveTrainController[0] = humanController.linearDriveProfile(-(((-gamepad1.right_stick_x) * (Math.abs(
                    -gamepad1.right_stick_x)) + ((-gamepad1.left_stick_x) * (Math.abs(-gamepad1.left_stick_x)))) / 2));
            driveTrainController[2] = humanController.linearDriveProfile(((-gamepad1.right_stick_y) - (-gamepad1.left_stick_y)) / 2);
        }
        else {//normal
            driveTrainController[1] = humanController.linearDriveProfile(-gamepad1.left_stick_y);
            driveTrainController[0] = humanController.linearDriveProfile(gamepad1.left_stick_x);
            driveTrainController[2] = humanController.linearDriveProfile(-gamepad1.right_stick_x);
        }

        if (gamepad1.left_bumper) {//now it is a reverse half power mode.

            driveTrainController[1] /= 3;///should be a bit slower if these changes work
            driveTrainController[0] /= 2;
            driveTrainController[2] /= 3;
            compensatedMecanumPowerDrive(driveTrainController[0],
                                         driveTrainController[1],
                                         driveTrainController[2],
                                         WEIGHT_COMP_RATIO);

            //I am trying a different drive mode without weight comp

        }
        else {
            mecanumPowerDrive(driveTrainController);
        }
    }

    private void runLifter(){
        liftPower    = liftProfile.limitWithoutAccel(liftMotor.getCurrentPosition(),
                                                     -gamepad2.right_stick_y);
        liftPosition = liftMotor.getCurrentPosition();
        liftMotor.setPower(liftPower);
    }

    private void runArm(){///TODO somethings up here, you need to rewrite the whole thing
        armPosition = armMotor.getCurrentPosition();


        if (armGoingToScoringPosition) {
            //go to scoring
            if (armHasArrived(armPosition, ARM_SCORING_POSITION)) {
                armHoldPosition           = armPosition;
                armGoingToScoringPosition = false;
                armPower                  = 0;
                //reset hold point
                //stop trying to go there
            }
            else {
                armPower = armProfile.RunToPositionWithAccel(initialArmPosition,
                                                             armPosition,
                                                             ARM_SCORING_POSITION);
            }
        }
        else if ((liftPower != 0) && armPosition < 200 && gamepad2.left_stick_y <= 0) {
            armPower = 0.1;
        }
        else if (armGoingToHomePosition) {
            //go to home
            if (armHasArrived(armPosition, ARM_HOME_POSITION)) {
                armHoldPosition        = armPosition;
                armGoingToHomePosition = false;
                armPower               = 0;
                //reset hold point
                //stop trying to go there
            }
            else {
                armPower = armProfile.RunToPositionWithAccel(initialArmPosition,
                                                             armPosition,
                                                             ARM_HOME_POSITION);
            }

        }
        else {
            if (Math.abs(gamepad2.left_stick_y) < 0.1 && armPosition > 1000) {
                holdArmUp();
            }
            else {
                armHoldPosition = armPosition;
                armPower        = armProfile.limitWithoutAccel(armPosition, gamepad2.left_stick_y);
            }
        }


        armMotor.setPower(armPower);
    }

    private void getPositionTargets(){
        //this code checks to see if we are going to a new target, and of so changes the desired direction and resets the initial position
        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            armGoingToScoringPosition = false;
            armGoingToHomePosition    = false;
        }
        else if (gamepad2.dpad_down) {//this one makes the arm go up
            armGoingToScoringPosition = true;
            armGoingToHomePosition    = false;
            //liftGoingToHomePosition = false;
            initialArmPosition = armMotor.getCurrentPosition();
        }
        else if (gamepad2.dpad_up) {//this one makes the lifter go down and the arm go home (basically a reset)
            armGoingToHomePosition = true;
            //liftGoingToHomePosition = true;
            armGoingToScoringPosition = false;
            initialArmPosition        = armMotor.getCurrentPosition();
            //initialLiftPosition = liftMotor.getCurrentPosition();
        }
    }

    private void runIntake(){
        intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        setIntakePower(intakePower);
    }

    private void runServos(){
        latchIsDown.update(gamepad1.x);
        markerDropper.update(gamepad2.a);
        //grabberIsEngaged.update(gamepad2.right_bumper);

        //peter gets full control
        if (armGoingToHomePosition) openGrabber();
        else if (gamepad2.y) closeGrabber();
        else if (gamepad2.right_bumper) openGrabber();

        if (!latchIsDown.get()) openLatch();
        else closeLatch();

        if (markerDropper.get()) holdMarker();
        else dropMarker();


        //Retract odometry
        if (runtime.milliseconds() < 1000) {
            odometryRetractServo.setPosition(0.75);
        }
        else {
            odometryRetractServo.setPosition(0.5);
        }
    }

    private void holdArmUp(){
        if (armPosition > armHoldPosition) {
            armPower = -0.4;
        }
        else {
            armPower = 0;
        }
    }


}
