package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.*;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.*;
import static org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode.GRABBER_POSITION.STOWED;

/**
 * Perhaps try implementing optimized rev drive
 */
public class DWAIAutonomous {


    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");
    private double BLOCK_OFFSET_X_POSITION = propertiesLoader.getDoubleProperty("BLOCK_OFFSET_X_POSITION");
    private double BLOCK_OFFSET_X_POSITION_FIRST = propertiesLoader.getDoubleProperty("BLOCK_OFFSET_X_POSITION_FIRST");
    private double BLOCK_OFFSET_Y_POSITION = propertiesLoader.getDoubleProperty("BLOCK_OFFSET_Y_POSITION");
    private double BLOCK_Y_POSITION = propertiesLoader.getDoubleProperty("BLOCK_Y_POSITION");
    private double BRIDGE_Y_POSITION = propertiesLoader.getDoubleProperty("BRIDGE_Y_POSITION");
    private double INITIAL_FOUNDATION_X_POSITION = propertiesLoader.getDoubleProperty("INITIAL_FOUNDATION_X_POSITION");
    private double FOUNDATION_Y_POSITION = propertiesLoader.getDoubleProperty("FOUNDATION_Y_POSITION");
    private double DEPOT_Y_POSITION = propertiesLoader.getDoubleProperty("DEPOT_Y_POSITION");
    private double START_OF_FOUNDATION_X = propertiesLoader.getDoubleProperty("START_OF_FOUNDATION_X");
    private double END_OF_BRIDGE_STONE_X = propertiesLoader.getDoubleProperty("END_OF_BRIDGE_STONE_X");
    private double OPEN_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("OPEN_LATCH_SERVO_POSITION");
    private double CLOSE_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("CLOSE_LATCH_SERVO_POSITION");
    private boolean PAUSE_STEPS = propertiesLoader.getBooleanProperty("PAUSE_STEPS");
    private boolean DEPLOY_LIFTER = propertiesLoader.getBooleanProperty("DEPLOY_LIFTER");
    private double LATERAL_MULTIPLIER = propertiesLoader.getDoubleProperty("LATERAL_MULTIPLIER");
    private int checks = 0;
    private StageSwitchingPipeline pipeline;
    private OpenCvCamera phoneCam;
    private SampleMecanumDriveREV drive;
    private RobotLinearOpMode robot;
    private FOUNDATION_ORIENTATION foundationOrientation;
    private PARK_POSITION parkPosition;
    private SIDE side;
    private ALLIANCE alliance;
    private SKYSTONE_POSITION skystone_position;
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private double startAngle = 0;
    private int blocksPlaced = 0;
    private volatile boolean deployed = false;

    public DWAIAutonomous(
            FOUNDATION_ORIENTATION foundationOrientation,
            PARK_POSITION parkPosition,
            SIDE side,
            ALLIANCE alliance,
            LinearOpMode opMode
    ){
        this.foundationOrientation = foundationOrientation;
        this.parkPosition          = parkPosition;
        this.side                  = side;
        this.alliance              = alliance;
        this.opMode                = opMode;
        this.telemetry             = opMode.telemetry;
    }

    private void print(String printString){
        opMode.telemetry.addLine(printString);
        opMode.telemetry.update();

        if (PAUSE_STEPS) {
            robot.stopAllMotors();
            //Ensures button is pressed and released before continuing
            while(opMode.opModeIsActive() && !opMode.gamepad1.a) ;
            while(opMode.opModeIsActive() && opMode.gamepad1.a) ;
        }

    }

    private void setupVariables(){

        if (alliance == ALLIANCE.RED) {
            BLOCK_Y_POSITION *= -1;
            BLOCK_OFFSET_Y_POSITION *= -1;
            BRIDGE_Y_POSITION *= -1;
            FOUNDATION_Y_POSITION *= -1;
            DEPOT_Y_POSITION *= -1;
        }

    }

    //TODO: incorporate that into the past trajectory %5
    private void strafeTo(double x_pos, double y_pos, double additionalStrafe){
        //check if turn necessary
        /*double turnAngle = drive.getPoseEstimate().getHeading();

        if(turnAngle > Math.PI){
            turnAngle -= 2 * Math.PI;
        }

        drive.turnSync(-turnAngle);*/

        if (additionalStrafe != 0) {
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(new Vector2d(x_pos, y_pos)).strafeRight(additionalStrafe).build());
        }
        else {
            drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(new Vector2d(x_pos, y_pos)).build());
        }

    }

    public void runOpMode(){
        robot = new RobotLinearOpMode(opMode);
        drive = new SampleMecanumDriveREV(opMode.hardwareMap);
        setupVariables();

        //Additional setup
        robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        openCVinit();

        telemetry.addLine("Initialized!");
        telemetry.update();

        opMode.waitForStart();

        //Some roadrunner stuff
        robot.setLeftGrabberPosition(DEFAULT);
        robot.setRightGrabberPosition(DEFAULT);
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);

        if (alliance == ALLIANCE.RED && side == SIDE.BLOCK) {
            drive.setPoseEstimate(new Pose2d(-34, -63, Math.toRadians(270)));
            startAngle = 270;
        }
        else if (alliance == ALLIANCE.RED && side == SIDE.FOUNDATION) {
            drive.setPoseEstimate(new Pose2d(15, -63, Math.toRadians(270)));
            startAngle = 270;
        }
        else if (alliance == ALLIANCE.BLUE && side == SIDE.BLOCK) {
            drive.setPoseEstimate(new Pose2d(-34, 63, Math.toRadians(90)));
            startAngle = 90;
        }
        else if (alliance == ALLIANCE.BLUE && side == SIDE.FOUNDATION) {
            drive.setPoseEstimate(new Pose2d(15, 63, Math.toRadians(90)));
            startAngle = 90;
        }

        if (side == SIDE.BLOCK) {
            executeBlockAuto();
        }
        else if (side == SIDE.FOUNDATION) {
            executeFoundationAuto();
        }
        else if (side == SIDE.QUICK_PARK) {
            //Deploy lifter
            Thread deploy = new Thread(() -> {
                startDeploy();
                deployed = true;
            });

            deploy.start();
            while(!deployed && opMode.opModeIsActive()) ;

            drive.followTrajectorySync(drive.trajectoryBuilder().forward(18).build());

        }
        else if (side == SIDE.SLOW_PARK) {
            //Deploy lifter
            Thread deploy = new Thread(() -> {
                startDeploy();
                deployed = true;
            });

            deploy.start();
            opMode.sleep(25000);

            drive.followTrajectorySync(drive.trajectoryBuilder().forward(18).build());

        }

        //AutoTransitioner.transitionOnStop(opMode, "Skystone Main Teleop", alliance);
    }

    private void executeBlockAuto(){
        //Detect block
        while(skystone_position == null || checks < 10){
            getSkyStonePosition();
            checks++;
            opMode.sleep(50);
        }
        //phoneCam.closeCameraDevice();

        robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);

        //grabber thread


        //Deploy lifter
        Thread deployThread = new Thread(() -> {
            startDeploy();
            deployed = true;
        });

        deployThread.start();
        setGrabberPos(GRABBING_SOON);

        switch (skystone_position) {
            case ONE_AND_FOUR:
                //Block 1 position
                grabFirstBlock(1);
                break;
            case TWO_AND_FIVE:
                //Block 2 position
                grabFirstBlock(2);
                break;
            case THREE_AND_SIX:
                //Block 3 position
                grabFirstBlock(3);
                break;
        }

        while(!deployed && opMode.opModeIsActive());

        Thread grabberThread = new Thread(() -> {
            while(drive.getPoseEstimate().getX() < END_OF_BRIDGE_STONE_X){
            }
            setGrabberPos(DROPPING_SOON);
        });

        grabberThread.start();

        placeBlock();

        grabberThread = new Thread(() -> {
            while(drive.getPoseEstimate().getX() > START_OF_FOUNDATION_X){
            }
            setGrabberPos(DEFAULT);
            while(drive.getPoseEstimate().getX() > END_OF_BRIDGE_STONE_X){
            }
            setGrabberPos(GRABBING_SOON);
        });

        grabberThread.start();

        switch (skystone_position) {
            case ONE_AND_FOUR:
                grabBlock(1);
                break;
            case TWO_AND_FIVE:
                grabBlock(2);
                break;
            case THREE_AND_SIX:
                grabBlock(3);
                break;
        }

        grabberThread = new Thread(() -> {
            while(drive.getPoseEstimate().getX() < END_OF_BRIDGE_STONE_X){
            }
            setGrabberPos(DROPPING_SOON);
        });

        grabberThread.start();

        placeBlock();

        grabberThread = new Thread(() -> {
            double heading = drive.getPoseEstimate().getHeading();
            while(Math.abs(drive.getPoseEstimate().getHeading() - heading) < Math.toRadians(5)){
            }
            setGrabberPos(DEFAULT);
        });

        grabberThread.start();

        dragFoundation();

        //grab more block?

        telemetry.addLine("Done!");
        telemetry.update();
    }

    private void grabFirstBlock(int index){

        if(-36 - (8 * index) + BLOCK_OFFSET_X_POSITION_FIRST > -48){
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .splineTo(new Pose2d(-48, BLOCK_OFFSET_Y_POSITION, 0))
                            .reverse()
                            .splineTo(new Pose2d(-36 - 8 * index + BLOCK_OFFSET_X_POSITION_FIRST, BLOCK_OFFSET_Y_POSITION, 0))
                            .build());
        } else{
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .reverse()
                            .splineTo(new Pose2d(-48, BLOCK_OFFSET_Y_POSITION, 0))
                            .splineTo(new Pose2d(-36 - 8 * index + BLOCK_OFFSET_X_POSITION_FIRST, BLOCK_OFFSET_Y_POSITION, 0))
                            .build());
        }

        strafeTo(-36 - 8 * index + BLOCK_OFFSET_X_POSITION_FIRST, BLOCK_Y_POSITION, 0 * Math.signum(BLOCK_Y_POSITION));

        setGrabberPos(GRABBING);
        opMode.sleep(100);
        setGrabberPos(STOWED);
        opMode.sleep(100);
    }

    private void grabBlock(int index){
        print("Going back for another one");
        setGrabberPos(READY);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .reverse()
                        .splineTo(new Pose2d(6, BRIDGE_Y_POSITION, 0))
//                        .splineTo(new Pose2d(-6, BRIDGE_Y_POSITION, 0))
                        .splineTo(new Pose2d(-12 - index * 8 + BLOCK_OFFSET_X_POSITION, BLOCK_Y_POSITION, 0))
                        .build());
        strafeTo(-12 - index * 8 + BLOCK_OFFSET_X_POSITION, BLOCK_Y_POSITION, 2.5 * Math.signum(BLOCK_Y_POSITION));
        print("Grabbing block");

        //        setGrabberPos(GRABBING);
        //        opMode.sleep(GRAB_DELAY);
        //        setGrabberPos(STOWED);
        //        opMode.sleep(GRAB_DELAY);

        setGrabberPos(GRABBING);
        opMode.sleep(100);
        setGrabberPos(STOWED);
        opMode.sleep(100);
    }

    private void dragFoundation(){
        print("Latching foundation");

        drive.turnSync(Math.toRadians(180 - startAngle));
        drive.followTrajectorySync(drive.trajectoryBuilder().strafeTo(new Vector2d(50, FOUNDATION_Y_POSITION)).build());

        while(!robot.getFoundationSensorPressed()){
            robot.mecanumPowerDrive(0, -0.3, 0);
            drive.updatePoseEstimate();
        }

        robot.mecanumPowerDrive(0, 0, 0);
        robot.setLatchPosition(CLOSE_LATCH_SERVO_POSITION);

        opMode.sleep(100);

        //WILLIAM
        //the angles are in radians, limited to between 0, 2pi, positive only
        //connect using 192.168.49.1:8080/dash, switch graph to field from Default to see positioning
        //roadrunner assumes lateral distance = forward/back distance, added a lateral multiplier
        //accessible using strafeto method, which just strafes left/right along the y axis only
        //check the open/close latch positions

        drive.followTrajectorySync(drive.trajectoryBuilder().splineTo(new Pose2d(45, DEPOT_Y_POSITION, Math.toRadians(startAngle + 45 * Math.signum(DEPOT_Y_POSITION)))).build());

        /*drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(21, DEPOT_Y_POSITION, Math.toRadians(180)))
                        //.reverse()
                        //.splineTo(new Pose2d(45, DEPOT_Y_POSITION, Math.toRadians(180)))
                        .build()
        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(30, 63 * Math.signum(DEPOT_Y_POSITION)))
                        .reverse()
                        .splineTo(new Pose2d(63, 63 * Math.signum(DEPOT_Y_POSITION), Math.toRadians(180)))
                        .build()
        );*/

        double turnAngle = drive.getPoseEstimate().getHeading() - Math.PI;

        if (turnAngle > Math.PI) {
            turnAngle -= 2 * Math.PI;
        }

        drive.turnSync(-turnAngle);

        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);
        opMode.sleep(100);

        drive.followTrajectorySync(drive.trajectoryBuilder().back(5).strafeLeft((DEPOT_Y_POSITION - BRIDGE_Y_POSITION) * LATERAL_MULTIPLIER).splineTo(new Pose2d(0, BRIDGE_Y_POSITION, Math.toRadians(180))).build());
    }

    private void placeBlock(){
        print("Going under bridge");

        if (blocksPlaced == 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            //.strafeLeft((BRIDGE_Y_POSITION - BLOCK_Y_POSITION))
                            .splineTo(new Pose2d(-8, BRIDGE_Y_POSITION, 0))
                            .splineTo(new Pose2d(2, BRIDGE_Y_POSITION, 0))
                            .splineTo(new Pose2d(INITIAL_FOUNDATION_X_POSITION + blocksPlaced * 8, FOUNDATION_Y_POSITION, 0))
                            .build());
            //strafeTo(INITIAL_FOUNDATION_X_POSITION + blocksPlaced * 8, FOUNDATION_Y_POSITION, 0);
        }
        else {

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeLeft((BRIDGE_Y_POSITION - BLOCK_Y_POSITION))
                            .splineTo(new Pose2d(0, BRIDGE_Y_POSITION, 0))
//                           .splineTo(new Pose2d(-8, BRIDGE_Y_POSITION, 0))
                            .splineTo(new Pose2d(INITIAL_FOUNDATION_X_POSITION + blocksPlaced * 8, FOUNDATION_Y_POSITION, 0))
                            .build());
            //strafeTo(INITIAL_FOUNDATION_X_POSITION + blocksPlaced * 8, FOUNDATION_Y_POSITION, 0);
        }

        print("Placing block");

        blocksPlaced++;
        setGrabberPos(DROPPING);
        opMode.sleep(50);
    }

    private void executeFoundationAuto(){

        //make heading a variable
        // make y positions a variable

        //strafe to 40, 63
        //back up to ~40, 35
        //latch
        //turn to 180
        //strafe to 40, 63
        //unlatch
        //park at 0, 63
    }

    private void setGrabberPos(GRABBER_POSITION pos){

        switch (alliance) {
            case BLUE:
                robot.setRightGrabberPosition(pos);
                break;
            case RED:
                robot.setLeftGrabberPosition(pos);
                break;
        }

    }

    private void startDeploy(){

        if (!DEPLOY_LIFTER) {
            return;
        }

        robot.setLiftPower(-0.5);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.setLiftPower(0);

        robot.setArmPower(.2);
        try {
            sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setArmPower(0);

        robot.setLiftPower(0.2);
        robot.setArmPower(-.2);

        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setLiftPower(0);
        robot.setArmPower(0);

        robot.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);

        try {
            sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setLiftRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setLiftRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.setArmRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void getSkyStonePosition(){
        int valLeft;
        int valMid;
        int valRight;

        int[] vals = pipeline.getPositions();
        valLeft  = vals[0];
        valMid   = vals[1];
        valRight = vals[2];

        if (alliance == ALLIANCE.BLUE) {
            if (valLeft == 0) skystone_position = SKYSTONE_POSITION.THREE_AND_SIX;
            if (valMid == 0) skystone_position = SKYSTONE_POSITION.TWO_AND_FIVE;
            if (valRight == 0) skystone_position = SKYSTONE_POSITION.ONE_AND_FOUR;
        }
        else {
            if (valLeft == 0) skystone_position = SKYSTONE_POSITION.ONE_AND_FOUR;
            if (valMid == 0) skystone_position = SKYSTONE_POSITION.TWO_AND_FIVE;
            if (valRight == 0) skystone_position = SKYSTONE_POSITION.THREE_AND_SIX;
        }

        if (skystone_position == null) {
            //throw new RuntimeException("No skystone detected!");
            telemetry.addLine("No Skystone detected!");
            telemetry.update();
        }
        else {
            telemetry.addLine(skystone_position.name());
            telemetry.update();
        }

    }

    private void openCVinit(){
        float offsetX         = propertiesLoader.getFloatProperty("OFFSET_X");//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float allianceOffsetX = propertiesLoader.getFloatProperty("ALLIANCE_OFFSET_X");//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
        float offsetY         = propertiesLoader.getFloatProperty("OFFSET_Y");//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
        float distScale       = propertiesLoader.getFloatProperty("DIST_SCALE");//changing this scales the distance between everything by a factor

        if (alliance == ALLIANCE.BLUE) {
            offsetX -= allianceOffsetX;
        }

        //offset array: 0 = x, 1 = y, 2 = distScale
        float[] offsetArray = {offsetX, offsetY, distScale};
        pipeline = new StageSwitchingPipeline(offsetArray);

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(pipeline);//different stages
        int rows = 640;
        int cols = 480;
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//stream on phone
    }

    enum SKYSTONE_POSITION {
        ONE_AND_FOUR,
        TWO_AND_FIVE,
        THREE_AND_SIX,
    }

    public enum FOUNDATION_ORIENTATION {
        HORIZONTAL,
        VERTICAL
    }

    public enum PARK_POSITION {
        WALL,
        BRIDGE
    }

    public enum ALLIANCE {
        RED,
        BLUE
    }

    public enum SIDE {
        BLOCK,
        FOUNDATION,
        QUICK_PARK,
        SLOW_PARK
    }

}
