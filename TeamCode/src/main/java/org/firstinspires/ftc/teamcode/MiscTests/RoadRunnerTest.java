/*package org.firstinspires.ftc.teamcode.MiscTests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DWAIAutonomous;
import org.firstinspires.ftc.teamcode.IO.PropertiesLoader;
import org.firstinspires.ftc.teamcode.Robot.RobotLinearOpMode;
import org.firstinspires.ftc.teamcode.Robot.StageSwitchingPipeline;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * IMPORTANT: THIS AUTO ASSUMES BLUE ALLIANCE (for now)
 * TODO: replace DWAIAutonomous class with this one sooner or later
 */
/*public class DWAIAutonomous {

    private RobotLinearOpMode robot;
    private volatile boolean flag = false;

    private PropertiesLoader propertiesLoader = new PropertiesLoader("Autonomous");
    private double OPEN_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("OPEN_LATCH_SERVO_POSITION");
    private double CLOSE_LATCH_SERVO_POSITION = propertiesLoader.getDoubleProperty("CLOSE_LATCH_SERVO_POSITION");

    private int checks = 0;
    private StageSwitchingPipeline pipeline;

    private RobotLinearOpMode robot;
    private FOUNDATION_ORIENTATION foundationOrientation;
    private PARK_POSITION parkPosition;
    private SIDE side;
    private ALLIANCE alliance;
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private SKYSTONE_POSITION skystone_position;

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

    public void runOpMode(){
        robot = new RobotLinearOpMode(opMode);
        SampleMecanumDriveREV drive = new SampleMecanumDriveREV(opMode.hardwareMap);

        //Additional setup
        robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        openCVinit();

        telemetry.addLine("Initialized!");
        telemetry.update();

        opMode.waitForStart();

        //Some roadrunner stuff
        robot.setLeftGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.DEFAULT);
        robot.setRightGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.DEFAULT);
        robot.setLatchPosition(OPEN_LATCH_SERVO_POSITION);

        if(alliance == ALLIANCE.RED && side == SIDE.BLOCK){
            drive.setPoseEstimate(new Pose2d(-34, -63, Math.toRadians(270)));
        } else if(alliance == ALLIANCE.RED && side == SIDE.FOUNDATION){
            drive.setPoseEstimate(new Pose2d(15, -63, Math.toRadians(270)));
        } else if(alliance == ALLIANCE.BLUE && side == SIDE.BLOCK){
            drive.setPoseEstimate(new Pose2d(-34, 63, Math.toRadians(90)));
        } else if(alliance == ALLIANCE.BLUE && side == SIDE.FOUNDATION){
            drive.setPoseEstimate(new Pose2d(15, 63, Math.toRadians(90)));
        }

        if(side == SIDE.BLOCK) {

            //Detect block
            while (skystone_position == null || checks < 10) {
                getSkyStonePosition();
                checks++;
                opMode.sleep(100);
            }

        }

        robot.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(12)
                        .build()
        );

        Thread deploy = new Thread(() -> {
            startDeploy();
            flag = true;
        });

        deploy.start();

        drive.turnSync(0);

        robot.setRightGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.READY);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-20, 33.6, 0))
                        .build()
        );

        robot.setRightGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.GRABBING);
        opMode.sleep(500);
        robot.setRightGrabberPosition(RobotLinearOpMode.GRABBER_POSITION.STOWED);
        opMode.sleep(500);

        while(!flag);

        /*drive.followTrajectory(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(0, 40, 0))
                        .splineTo(new Pose2d(45, 35, 0))
                        .build()
        );

        while(drive.isBusy() && opModeIsActive()){
            drive.update();
        }*/

/*        telemetry.addLine("Done!");
        telemetry.update();

        //X diffs
        //POS 1: -20, 33.3
        //POS 2: -28, 33.3
        //POS 3: -36, 33.3

        //BRIDGE: 0, 40 < go here between blocks
        //FOUNDATION: 45, 35

        //X diffs
        //POS 4: -44, 33.3
        //POS 5: -52, 33.3
        //POS 6: -60, 33.3

        //turn to 90 at end
        //move to 60, 63
        //park at 0, 38

    }

    private void startDeploy(){
        robot.setLiftPower(-0.5);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        robot.setLiftPower(0);

        robot.setArmPower(.2);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setArmPower(0);

        robot.setLiftPower(0.2);
        robot.setArmPower(-.2);

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.setLiftPower(0);
        robot.setArmPower(0);

        robot.setLiftZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.setArmZeroPowerProperty(DcMotor.ZeroPowerBehavior.FLOAT);

        try {
            Thread.sleep(2000);
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
        valLeft = vals[0];
        valMid = vals[1];
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
        float distScale       = propertiesLoader.getFloatProperty("DIST_SCALE");

        //if (alliance == DWAIAutonomous.ALLIANCE.BLUE) {
            offsetX -= allianceOffsetX;
        //}

        //offset array: 0 = x, 1 = y, 2 = distScale
        float[] offsetArray = {offsetX, offsetY, distScale};
        pipeline = new StageSwitchingPipeline(offsetArray);

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        OpenCvCamera phoneCam            = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(pipeline);//different stages
        int rows = 640;
        int cols = 480;
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.
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
*/