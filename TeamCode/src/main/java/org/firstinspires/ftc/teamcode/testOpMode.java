////imports the package so that all the code can be used
//package org.firstinspires.ftc.teamcode;
//
//
////all the imports that come from qualcomm, mainly needing to do with the Hardware and the phone running
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
////all the imports that come from Casey and I in the Package
//import org.firstinspires.ftc.teamcode.Control.BallisticMotionProfile;
//import org.firstinspires.ftc.teamcode.Control.EverHit;
//import org.firstinspires.ftc.teamcode.Control.HumanController;
//import org.firstinspires.ftc.teamcode.Control.Toggle;
//import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;
//
////all the imports that come from java, abs is absolute value
//import static java.lang.Math.abs;
//
////name that appears on the phone and the group that it is a part of
//@TeleOp(name = "Test OpMode", group = "Linear Opmode")
//
////class name that corresponds with the file, it is a linear opMode that I then changed
//public class testOpMode extends LinearOpMode {
//
//
//    /*Mini Lesson:
//    Public - Can be accessed from anywhere
//    Private - Can be accessed from within the class
//    *Nothing* - Can be accessed from within the package (package private)
//    Proteccted - Subclasses can access the object (think extends)
//     */
//
//    //We start with some arm positions that we will go to in the future
//    //this one is where we start, with the arm at 0 resting in the robot.
//    private final double ARM_DOWN_POSITION = 0;
//    //this one is all
//    private final double ARM_UP_POSITION = 1600;
//
//
//    final double LIFT_HOME_POSITION = 0;//for controlling the lifter
//
//    //so these exist so that I do not get confused, not strictly necessary, 0 is the strafe, 1 is the forward, and 2 is the rotation
//    private double[] driveTrainController = new double[3];
//    /*Mini Lesson:
//    Int - Whole numbers
//    Long - Long whole numbers
//    Float - Whole numbers but hexidecimal(base 16)
//    Double - Floating point (decimal values)
//    Char - Character, the basis of a word
//    Boolean - True/False
//     */
//
//
//    //public BallisticMotionProfile liftProfile = new BallisticMotionProfile(0, 3300, 1000, 0.25, 1, .7);
//    //public BallisticMotionProfile armProfile = new BallisticMotionProfile(topArmEncoder, bottomArmEncoder, 1000, 0.25, 1, .7);
//
//    private double intakePower = 0;
//    private double armPower = 0;
//    private double liftPower = 0;
//
//    /////////////////Casey's Position Shenanegins - making some runtoposition commands
//    //sensor values, also exist to make the code cleaner
//    double armPosition = 0;
//    double liftPosition;
//
//    /////////////////Casey's Position Shenanegins - making some runtoposition commands
//
//    //these are used to determine whether or not we are actively trying to go to a certain preset position
//    //as determined by user input
//    boolean armGoingToScoringPosition = false;
//    boolean armGoingToHomePosition = false;
//    boolean liftGoingToHomePosition = false;//added new
//
//    //now we make a variable to use later which represents the initial position when doing a runtoposition command
//    double initialArmPosition = 0;
//    double initialLiftPosition = 0;
//
//    /////////////////////////
//
//    boolean blockIntakeTouchSensor;
//    RobotLinearOpMode robotLinearOpMode;
//    Toggle latchToggle;
//    Toggle grabberToggle;
//    EverHit blockEverInIntake = new EverHit();
//    Toggle driveMode;
//
//    HumanController humanController = new HumanController(0.1, 1);
//    //the amount of time that the program has run
//    private ElapsedTime runtime = new ElapsedTime();
//
//    //PROPERTIES CONSTANTS
//    PropertiesLoader propertiesLoader = new PropertiesLoader("TeleOp");
//    //this is the ratio by which the strafe power to the back wheels is multiplied by
//    final double WEIGHT_COMP_RATIO = propertiesLoader.getDoubleProperty("WEIGHT_COMPENSATION_RATIO");
//    //We start with some arm positions that we will go to in the future
//    //this one is where we start, with the arm at 0 resting in the robot.
//    final double ARM_HOME_POSITION = propertiesLoader.getDoubleProperty("ARM_HOME_POSITION");
//    ;
//    //this is the socring position for the arm. not quite fully behind the robot, but as high as possible while still able to score
//    final double ARM_SCORING_POSITION = propertiesLoader.getDoubleProperty("ARM_SCORING_POSITION");
//    ;
//
//    //this is the loop that repeats until the end of teleOp.
//    @Override
//    public void runOpMode() {
//        robotLinearOpMode = new RobotLinearOpMode(this);
//
//        Toggle latchIsDown = new Toggle(false);
//        Toggle grabberIsEngaged = new Toggle(false);
//
//        Toggle driveMode = new Toggle(false);
//        Toggle markerDropper = new Toggle(true);
//
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        //this is when the robot receives the 'play' command
//        waitForStart();
//        runtime.reset();
//
//        robotLinearOpMode.setAllMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robotLinearOpMode.setDriveTrainRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robotLinearOpMode.setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robotLinearOpMode.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        robotLinearOpMode.setAllMotorZeroPowerProperty(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//        while (opModeIsActive()) {
//
//            //sets up the condidtion for the drivetrain
//            driveMode.update(gamepad1.right_bumper);
//
//            if (driveMode.get()) {
//                driveTrainController[1] = humanController.linearDriveProfile(((-gamepad1.right_stick_y) * (abs(-gamepad1.right_stick_y)) + ((-gamepad1.left_stick_y) * (abs(-gamepad1.left_stick_y)))) / 2);
//                driveTrainController[0] = humanController.linearDriveProfile(-(((-gamepad1.right_stick_x) * (abs(-gamepad1.right_stick_x)) + ((-gamepad1.left_stick_x) * (abs(-gamepad1.left_stick_x)))) / 2));
//                driveTrainController[2] = humanController.linearDriveProfile(((-gamepad1.right_stick_y) - (-gamepad1.left_stick_y)) / 2);
//            } else {
//                driveTrainController[1] = humanController.linearDriveProfile(-gamepad1.left_stick_y);
//                driveTrainController[0] = humanController.linearDriveProfile(gamepad1.left_stick_x);
//                driveTrainController[2] = humanController.linearDriveProfile(-gamepad1.right_stick_x);
//            }
//
//            if (gamepad1.left_bumper) {
//                driveTrainController[1] /= 3;
//                driveTrainController[0] /= 2;
//                driveTrainController[2] /= 3;
//                robotLinearOpMode.compensatedMecanumPowerDrive(driveTrainController[0], driveTrainController[1], driveTrainController[2], WEIGHT_COMP_RATIO);
//            } else {
//                robotLinearOpMode.mecanumPowerDrive(driveTrainController);
//            }
//
//            //increments the intake power
//            intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
//
//            ///////////The second part of Casey's arm thing
//            if (liftGoingToHomePosition) {
//
//                if (liftHasArrived(robotLinearOpMode.getArmEncoder(), LIFT_HOME_POSITION)) {
//                    liftGoingToHomePosition = false;
//                    liftPower = 0;
//                } else
//                    liftPower = robotLinearOpMode.liftProfile.RunToPositionWithAccel(initialLiftPosition, robotLinearOpMode.getLiftEncoder(), LIFT_HOME_POSITION);
//            } else {
//                //this s hould work but be careful
//                liftPower = robotLinearOpMode.liftProfile.limitWithoutAccel(robotLinearOpMode.getLiftEncoder(), gamepad2.right_stick_y);
//            }
//
//
//            if (armGoingToScoringPosition) {
//
//                if (armHasArrived(robotLinearOpMode.getArmEncoder(), ARM_SCORING_POSITION)) {
//                    armGoingToScoringPosition = false;
//                    armPower = 0;
//                } else
//                    armPower = robotLinearOpMode.armProfile.RunToPositionWithoutAccel(initialArmPosition, robotLinearOpMode.getArmEncoder(), ARM_SCORING_POSITION);
//
//            } else if (armGoingToHomePosition) {
//
//                if (armHasArrived(robotLinearOpMode.getArmEncoder(), ARM_HOME_POSITION)) {
//                    armGoingToHomePosition = false;
//                    armPower = 0;
//                } else
//                    armPower = robotLinearOpMode.armProfile.RunToPositionWithoutAccel(initialArmPosition, robotLinearOpMode.getArmEncoder(), ARM_HOME_POSITION);
//
//            } else {
//                armPower = robotLinearOpMode.armProfile.limitWithoutAccel(robotLinearOpMode.getArmEncoder(), gamepad2.left_stick_y);
//            }
//
//            //this code checks to see if we are going to a new target, and of so changes the desired direction and resets the initial position
//            if (gamepad2.dpad_down) {//this one makes the arm go up
//                armGoingToScoringPosition = true;
//                armGoingToHomePosition = false;
//                liftGoingToHomePosition = false;
//                initialArmPosition = robotLinearOpMode.getArmEncoder();
//            } else if (gamepad2.dpad_up) {//this one makes the lifter go down and the arm go home (basically a reset)
//                armGoingToHomePosition = true;
//                liftGoingToHomePosition = true;
//                armGoingToScoringPosition = false;
//                initialArmPosition = robotLinearOpMode.getArmEncoder();
//                initialLiftPosition = robotLinearOpMode.getLiftEncoder();
//            }
//
//            ////END CASEY'S THING
//
//            latchIsDown.update(gamepad2.x);
//            grabberIsEngaged.update(gamepad2.y);
//            markerDropper.update(gamepad2.a);
//
//            //Grabber update with new logic
//            if ((armPosition > ARM_HOME_POSITION) && (armPosition < ARM_SCORING_POSITION)) {
//                robotLinearOpMode.closeGrabber();
//            } else if (intakePower != 0) {
//                robotLinearOpMode.openGrabber();
//            } else {
//                if (grabberIsEngaged.get()) robotLinearOpMode.openGrabber();
//                else robotLinearOpMode.closeGrabber();
//            }
//
//            if (armPosition > ARM_HOME_POSITION && armPosition < ARM_SCORING_POSITION - 500) {
//                robotLinearOpMode.openGrabber();
//            } else if (intakePower != 0) {
//                robotLinearOpMode.closeGrabber();
//            } else if (armPosition < ARM_HOME_POSITION) {
//                robotLinearOpMode.openGrabber();
//            } else {
//                if (grabberIsEngaged.get()) robotLinearOpMode.closeGrabber();
//                else robotLinearOpMode.openGrabber();
//            }
//
//            if (latchIsDown.get()) robotLinearOpMode.closeLatch();
//            else robotLinearOpMode.openLatch();
//
//            if (markerDropper.get()) robotLinearOpMode.holdMarker();
//            else robotLinearOpMode.dropMarker();
//
//            //sends this to the motors.
//            robotLinearOpMode.setIntakePower(intakePower);
//            robotLinearOpMode.setArmPower(armPower);
//            robotLinearOpMode.setLiftPower(liftPower);
//
//            //gets the position arm encoder
//            armPosition = robotLinearOpMode.getArmEncoder();
//            liftPosition = robotLinearOpMode.getLiftEncoder();
//            //blockIntakeTouchSensor = robotLinearOpMode.isBlockInIntake();
//
//            sendTelemetry();
//        }
//    }
//
//    //tells us if the arm is on target
//    boolean armHasArrived(double current, double target) {
//        boolean arrived;
//
//        //make sure we made it depending on which way we came
//        if ((initialArmPosition < target) && (target < current)) {
//            arrived = true;
//        } else if ((initialArmPosition > target) && (target > current)) {
//            arrived = true;
//        } else {
//            arrived = false;
//        }
//        return arrived;
//    }
//
//
//    boolean liftHasArrived(double current, double target) {
//        boolean arrived;
//
//        //make sure we made it depending on which way we came
//        if ((initialLiftPosition < target) && (target < current)) {
//            arrived = true;
//        } else if ((initialLiftPosition > target) && (target > current)) {
//            arrived = true;
//        } else {
//            arrived = false;
//        }
//        return arrived;
//    }
//
//    void sendTelemetry() {
//        telemetry.addData("Arm Position: ", armPosition);
//        telemetry.addData("Lift Position: ", liftPosition);
//        telemetry.addData("Open Intake Touch Boolean", robotLinearOpMode.intakeIsOpen());
//        telemetry.update();
//    }
//}
//
