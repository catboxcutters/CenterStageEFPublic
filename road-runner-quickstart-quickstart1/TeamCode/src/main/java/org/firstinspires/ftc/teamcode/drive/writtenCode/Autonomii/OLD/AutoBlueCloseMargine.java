package org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.OLD;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.initPosition;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.writtenCode.CaseDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.AvionController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CataratController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.CollectForbarController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ExtenderController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ForbarOuttakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.IntakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.LiftMotorController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ParbrizController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TurretController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.RotateClawController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.ScoringController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.SigurantaOuttakeController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController;
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TubuleteController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(group = "Auto")
@Disabled
public class AutoBlueCloseMargine extends LinearOpMode {

    static Pose2d startPose = new Pose2d(10, 62, Math.toRadians(270));
    public static double forwardRight = 6.75;
    public static  double PRELOAD_LEFT_X = 16;
    public static  double PRELOAD_LEFT_Y = 43.5;
    public static  double PRELOAD_ANGLE_LEFT = -45;

    public static  double PLACE_SPIKE_LEFT_X = 48;
    public static  double PLACE_SPIKE_LEFT_Y = 46;
    public static  double ANGLE_SPIKE_LEFT = 0;

    public static  double PARK_LEFT_X = 43;
    public static  double PARK_LEFT_Y = 20;
    public static  double ANGLE_PARK_LEFT = 0;

    public static  double PRELOAD_MID_X = 12;
    public static  double PRELOAD_MID_Y = 37;
    public static  double PRELOAD_ANGLE_MID = 270;

    public static  double PLACE_SPIKE_MID_X = 48;
    public static  double PLACE_SPIKE_MID_Y = 39;
    public static  double ANGLE_SPIKE_MID = 0;

    public static  double PARK_MID_X = 43;

    public static  double PARK_MID_Y = 20;
    public static  double ANGLE_PARK_MID = 0;

    public static  double PRELOAD_RIGHT_X = 6;
    public static  double PRELOAD_RIGHT_Y = 40;
    public static  double PRELOAD_ANGLE_RIGHT = -145;

    public static  double PLACE_SPIKE_RIGHT_X = 48;
    public static  double PLACE_SPIKE_RIGHT_Y = 32;
    public static double ANGLE_SPIKE_RIGHT = 0;

    public static double PARK_RIGHT_X = 43;
    public static double PARK_RIGHT_Y = 62;
    public static double ANGLE_PARK_RIGHT = 0;


    public static final double GO_TO_STACK_X = 20f;
    public static final double GO_TO_STACK_Y = 57.5f;
    public static final double LINE_TO_STACK_X = -30f;
    public static final double LINE_TO_STACK_Y = 57.5f;
    public static final double GO_TO_BACK_1_X = -55f;
    public static final double GO_TO_BACK_1_Y =55.5f;
    public static final double GO_TO_BACK_2_X = 10f;
    public static final double GO_TO_BACK_2_Y =55.5f;
    public static final double ANGLE_COLECTARE=40f;
    public static final double COLLECT_STACK_X_CYCLE1_C2 = -45f;
    public static final double COLLECT_STACK_Y_CYCLE1_C2 = 50f;


    public static final double COLLECT_STACK_X_CYCLE2_C2 = -28f;

    public static final double GO_TO_STACK_ANGLE_CYCLE1_C1 = 0;
    public static final double GO_TO_STACK_ANGLE_CYCLE2_C1= 0;
    public static final double GO_TO_STACK_ANGLE_CYCLE3_C1 = 0;
    public static final double COLLECT_STACK_X_CYCLE1_C1 = -27.5f;
    public static final double COLLECT_STACK_Y_CYCLE1_C1 = 9.5f;
    public static final double COLLECT_STACK_X_CYCLE2_C1 = -28f;
    public static final double COLLECT_STACK_Y_CYCLE2_C1 = 7.5f;
    public static final double COLLECT_STACK_X_CYCLE3_C1 = -27f;
    public static final double COLLECT_STACK_Y_CYCLE3_C1 = 8.5f;


    public static final double GO_TO_STACK_ANGLE_CYCLE1_C2 = 0.45;
    public static final double GO_TO_STACK_ANGLE_CYCLE2_C2= 0.60;
    public static final double GO_TO_STACK_ANGLE_CYCLE3_C2 = 0;


    public static final double COLLECT_STACK_Y_CYCLE2_C2 = 7.5f;
    public static final double COLLECT_STACK_X_CYCLE3_C2 = -26.5f;
    public static final double COLLECT_STACK_Y_CYCLE3_C2 = 8.5f;

    public static final double GO_TO_STACK_ANGLE_CYCLE1_C3 = 0;
    public static final double GO_TO_STACK_ANGLE_CYCLE2_C3= 0.2;
    public static final double GO_TO_STACK_ANGLE_CYCLE3_C3 = 0;
    public static final double COLLECT_STACK_X_CYCLE1_C3 = -28f;
    public static final double COLLECT_STACK_Y_CYCLE1_C3 = 9.5f;
    public static final double COLLECT_STACK_X_CYCLE2_C3 = -28f;
    public static final double COLLECT_STACK_Y_CYCLE2_C3 = 8.5f;
    public static final double COLLECT_STACK_X_CYCLE3_C3 = -26.5f;
    public static final double COLLECT_STACK_Y_CYCLE3_C3 = 8.5f;


    public static final double PLACE_BB_LLH1_X = 20;
    public static final double PLACE_BB_LLH1_Y = 11;
    public static final int ANGLE_BB_LLH1 = 0;
    public static final double PLACE_BB_LLH2_X = 49f;
    public static final double PLACE_BB_LLH2_Y = 48f;
    public static final double ANGLE_BB_LLH2 = -25f;


    public static int timeout_1pixel = 3;
    public static float timeout_nopixel = 2.5f;
    public static float time_reverse_scurt =0f;
    public static float timeToDrop = 3f;
    private void wait(int ms) {
        try{
            Thread.sleep(ms);
        }
        catch (InterruptedException ignored) {
            //nu face nimic
        }
    }


    enum STROBOT
    {
        START,
        PLACE_PURPLE_PIXEL,
        WAIT_FOR_PURPLE_PIXEL,
        PLACE_SPIKE_BACKDROP,
        PARK,
        RETRACT_LIFT,
        GO_COLLECT_PIXELS,
        LEAVE_WITH_2_PIXELS,
        PLACE_STACK_PIXELS_BB,
        GO_TO_STACK_FIRST,
        END_AUTO,
        FAILSAFE_NO_PIXELS,
        COLLECT_FAILSAFE
    }
    public static double timePlacePixel = 0.5;
    public static double delayLift = 0.4;
    public static double waitTimeBackDrop = 0.3;
    public static double timeOpenSlides = 6.3;

    public static double liftStack= 4;
    public static boolean flag =true;
    boolean DID_FAILSAFE = false;
    ElapsedTime timerPunerePixel = new ElapsedTime();
    ElapsedTime timerLift = new ElapsedTime();
    ElapsedTime timerSlides = new ElapsedTime();
    ElapsedTime timerDrop = new ElapsedTime();
    ElapsedTime timeoutColectare = new ElapsedTime();
    ElapsedTime timerReverseScurt = new ElapsedTime();
    ElapsedTime AutoTimer = new ElapsedTime();
    CaseDetectionPipeline cameraRecognition;
    public static double cazAuto = 3;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);
//        robot.rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CataratController cataratController = new CataratController(robot);
        AvionController avionController = new AvionController(robot);
        IntakeController intakeController = new IntakeController(robot);
        CollectForbarController collectForbarController = new CollectForbarController(robot);
        TubuleteController tubuleteController = new TubuleteController(robot);
        TurretController turretController = new TurretController(robot);
        ParbrizController parbrizController = new ParbrizController(robot);
        SigurantaOuttakeController sigurantaOuttakeController = new SigurantaOuttakeController(robot);
        ForbarOuttakeController forbarOuttakeController = new ForbarOuttakeController(robot);
        RotateClawController rotateClawController = new RotateClawController(robot);
        ExtenderController extenderController = new ExtenderController(robot);
        LiftMotorController liftMotorController = new LiftMotorController(forbarOuttakeController,extenderController,robot,turretController);
        robot.forbarCutieIntake.setPosition(initPosition);
//        turretController.currentStatus = TurretController.Pixel2Status.OPEN;

        TransferController transferController = new TransferController(
                intakeController,tubuleteController,sigurantaOuttakeController, extenderController,robot);
        ScoringController scoringController = new ScoringController(turretController, sigurantaOuttakeController, parbrizController, rotateClawController);

        cataratController.update();
        avionController.update();
        extenderController.update(0);
        rotateClawController.update();
        forbarOuttakeController.update();
        liftMotorController.update();
        intakeController.update();
        collectForbarController.update();
        tubuleteController.update();
        transferController.update();
        turretController.update();
        parbrizController.update();
        sigurantaOuttakeController.update();
        scoringController.update();
//        cameraRecognition = new CaseDetectionPipeline(hardwareMap,telemetry,"blue");
//        cameraRecognition.initCamera();
//        cameraRecognition.start(1);


        Pose2d startPose = new Pose2d(10, 62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        //  TrajectoryVelocityConstraint VELLLH = getVelocityConstraint(40, 5, 12.05);
        double nrCycles = 0;
        double howManyCycles = 1;
        TrajectorySequence PLACE_PRELOAD_LEFT = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PRELOAD_LEFT_X, PRELOAD_LEFT_Y,Math.toRadians(PRELOAD_ANGLE_LEFT)))
                .lineTo(new Vector2d(14,53))
                .build();
        TrajectorySequence PLACE_SPIKE_LEFT = drive.trajectorySequenceBuilder(PLACE_PRELOAD_LEFT.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_LEFT_X, PLACE_SPIKE_LEFT_Y,Math.toRadians(ANGLE_SPIKE_LEFT)))
                .build();

        TrajectorySequence PARK_ROBOT_LEFT = drive.trajectorySequenceBuilder(PLACE_SPIKE_LEFT.end())
                .lineToLinearHeading(new Pose2d(PARK_LEFT_X, PARK_LEFT_Y,Math.toRadians(ANGLE_PARK_LEFT)))
                .strafeRight(5)
                .forward(10)
                .build();
//        TrajectorySequence GO_PLACE_ON_BACKBOARD = drive.trajectorySequenceBuilder(GO_COLLECT_STACK_CYCLE1.end())
//                .setReversed(false)
//                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y)) // se da cu spatele
//                .splineToConstantHeading(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y),Math.toRadians(0))
//                .build(); // te duce la backboard


        TrajectorySequence PLACE_PRELOAD_MID = drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(VELLLH)
                .lineToLinearHeading(new Pose2d(PRELOAD_MID_X, PRELOAD_MID_Y,Math.toRadians(PRELOAD_ANGLE_MID)))
                .build();
        TrajectorySequence PLACE_SPIKE_MID = drive.trajectorySequenceBuilder(PLACE_PRELOAD_MID.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_MID_X, PLACE_SPIKE_MID_Y,Math.toRadians(ANGLE_SPIKE_MID)))
                .build();


        TrajectorySequence PLACE_PRELOAD_RIGHT = drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(VELLLH)
                .lineToLinearHeading(new Pose2d(PRELOAD_RIGHT_X, PRELOAD_RIGHT_Y,Math.toRadians(PRELOAD_ANGLE_RIGHT)))
                .back(forwardRight)
                .build();
        TrajectorySequence PLACE_SPIKE_RIGHT = drive.trajectorySequenceBuilder(PLACE_PRELOAD_RIGHT.end())
                //  .setVelConstraint(VELLLH)
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_RIGHT_X, PLACE_SPIKE_RIGHT_Y,Math.toRadians(ANGLE_SPIKE_RIGHT)))
                .build();


        TrajectorySequence GO_TO_STACK_LEFT = drive.trajectorySequenceBuilder(PLACE_SPIKE_LEFT.end())
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y),Math.toRadians(180))
                .lineTo(new Vector2d(GO_TO_STACK_X,GO_TO_STACK_Y))
                .lineTo(new Vector2d(LINE_TO_STACK_X, LINE_TO_STACK_Y))
//                .lineToLinearHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .lineToSplineHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .build();

        TrajectorySequence GO_TO_STACK_MID = drive.trajectorySequenceBuilder(PLACE_SPIKE_MID.end())
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y),Math.toRadians(180))
                .lineTo(new Vector2d(GO_TO_STACK_X,GO_TO_STACK_Y))
                .lineTo(new Vector2d(LINE_TO_STACK_X, LINE_TO_STACK_Y))
//                .lineToLinearHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .lineToSplineHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .build();

        TrajectorySequence GO_TO_STACK_RIGHT = drive.trajectorySequenceBuilder(PLACE_SPIKE_RIGHT.end())
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y),Math.toRadians(180))
                .lineTo(new Vector2d(GO_TO_STACK_X,GO_TO_STACK_Y))
                .lineTo(new Vector2d(LINE_TO_STACK_X, LINE_TO_STACK_Y))
//                .lineToLinearHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .lineToSplineHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .build();
        TrajectorySequence GO_PLACE_ON_BACKBOARD = drive.trajectorySequenceBuilder(new Pose2d(COLLECT_STACK_X_CYCLE1_C2,COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .lineToLinearHeading(new Pose2d(GO_TO_BACK_1_X,GO_TO_BACK_1_Y,Math.toRadians(0)))
//                .lineTo(new Vector2d(LINE_TO_STACK_X,LINE_TO_STACK_Y))
                .lineTo(new Vector2d(GO_TO_BACK_2_X,GO_TO_BACK_2_Y))
                .lineToLinearHeading(new Pose2d(PLACE_BB_LLH2_X,PLACE_BB_LLH2_Y,Math.toRadians(ANGLE_BB_LLH2)))
//                .turn(Math.toRadians(-40))
//                .splineToConstantHeading(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y),Math.toRadians(0))
                // .lineTo(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y))
                .build(); // te duce la backboard

        TrajectorySequence GO_COLLECT_STACK_CYCLE1_C1 = drive.trajectorySequenceBuilder(
                        new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y,Math.toRadians(GO_TO_STACK_ANGLE_CYCLE1_C1)))
                .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_CYCLE1_C1, COLLECT_STACK_Y_CYCLE1_C1))
                .build();
        TrajectorySequence GO_PLACE_ON_BACKBOARD_CYCLE1_C1 = drive.trajectorySequenceBuilder(GO_COLLECT_STACK_CYCLE1_C1.end())
                .setReversed(false)
                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y )) // se da cu spatele
                .splineToConstantHeading(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y),Math.toRadians(0))
                // .lineTo(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y))
                .build(); // te duce la backboard

        TrajectorySequence GO_COLLECT_STACK_CYCLE1_C2 = drive.trajectorySequenceBuilder(
                        new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y,Math.toRadians(GO_TO_STACK_ANGLE_CYCLE1_C2)))
                .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2))
                .build();
        TrajectorySequence GO_PLACE_ON_BACKBOARD_CYCLE1_C2 = drive.trajectorySequenceBuilder(GO_COLLECT_STACK_CYCLE1_C2.end())
                .setReversed(false)
                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y )) // se da cu spatele
                .splineToConstantHeading(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y),Math.toRadians(0))
                //.lineTo(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y))
                .build(); // te duce la backboard

        TrajectorySequence GO_COLLECT_STACK_CYCLE1_C3 = drive.trajectorySequenceBuilder(
                        new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y,Math.toRadians(GO_TO_STACK_ANGLE_CYCLE1_C3)))
                .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_CYCLE1_C3, COLLECT_STACK_Y_CYCLE1_C3))
                .build();
        TrajectorySequence GO_PLACE_ON_BACKBOARD_CYCLE1_C3 = drive.trajectorySequenceBuilder(GO_COLLECT_STACK_CYCLE1_C3.end())
                .setReversed(false)
                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y )) // se da cu spatele
                .splineToConstantHeading(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y),Math.toRadians(0))
//                .lineTo(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y))
                .build(); // te duce la backboard


        TrajectorySequence GO_TO_STACK_CYCLE2_C1 = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD_CYCLE1_C1.end())
                .lineToLinearHeading(new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y, Math.toRadians(GO_TO_STACK_ANGLE_CYCLE2_C1)))
                .build();
        TrajectorySequence GO_COLLECT_STACK_CYCLE2_C1 = drive.trajectorySequenceBuilder(GO_TO_STACK_CYCLE2_C1.end())
                .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_CYCLE2_C1, COLLECT_STACK_Y_CYCLE2_C1))
                .build();
        TrajectorySequence GO_PLACE_ON_BACKBOARD_CYCLE2_C1 = drive.trajectorySequenceBuilder(GO_COLLECT_STACK_CYCLE2_C1.end())
                .setReversed(false)
                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y)) // se da cu spatele
                .splineToConstantHeading(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y),Math.toRadians(0))
//                .lineTo(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y))
                .build(); // te duce la backboard

        TrajectorySequence GO_TO_STACK_CYCLE2_C2 = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD_CYCLE1_C2.end())
                .lineToLinearHeading(new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y, Math.toRadians(GO_TO_STACK_ANGLE_CYCLE2_C2)))
                .build();
        TrajectorySequence GO_COLLECT_STACK_CYCLE2_C2 = drive.trajectorySequenceBuilder(GO_TO_STACK_CYCLE2_C2.end())
                .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_CYCLE2_C2, COLLECT_STACK_Y_CYCLE2_C2))
                .build();
        TrajectorySequence GO_PLACE_ON_BACKBOARD_CYCLE2_C2 = drive.trajectorySequenceBuilder(GO_COLLECT_STACK_CYCLE2_C2.end())
                .setReversed(false)
                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y)) // se da cu spatele
                .splineToConstantHeading(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y),Math.toRadians(0))
//                .lineTo(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y))
                .build(); // te duce la backboard

        TrajectorySequence GO_TO_STACK_CYCLE2_C3 = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD_CYCLE1_C3.end())
                .lineToLinearHeading(new Pose2d(GO_TO_STACK_X, GO_TO_STACK_Y, Math.toRadians(GO_TO_STACK_ANGLE_CYCLE2_C3)))
                .build();
        TrajectorySequence GO_COLLECT_STACK_CYCLE2_C3 = drive.trajectorySequenceBuilder(GO_TO_STACK_CYCLE2_C3.end())
                .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_CYCLE2_C3, COLLECT_STACK_Y_CYCLE2_C3))
                .build();
        TrajectorySequence GO_PLACE_ON_BACKBOARD_CYCLE2_C3 = drive.trajectorySequenceBuilder(GO_COLLECT_STACK_CYCLE2_C3.end())
                .setReversed(false)
                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y)) // se da cu spatele
                .splineToConstantHeading(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y),Math.toRadians(0))
//                .lineTo(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y))
                .build(); // te duce la backboard

        TrajectorySequence PARK_ROBOT = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD_CYCLE2_C1.end())
                .back(10)
                .lineToLinearHeading(new Pose2d(PARK_RIGHT_X, PARK_RIGHT_Y,Math.toRadians(ANGLE_PARK_RIGHT)))
                .build();

//        while (!isStarted()&&!isStopRequested())
//        {
//            int c =  cameraRecognition.getCase();
//            telemetry.addData("detected",c);
//            cazAuto=c;
//            telemetry.addLine("Init Complete");
//            telemetry.update();
//            sleep(50);
//        }
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested())
        {
            int extenderCurrentPosition = robot.rightExtension.getCurrentPosition();
            switch (status)
            {
                case START:
                {
                    AutoTimer.reset();
                    if (cazAuto == 1)
                    {
                        drive.followTrajectorySequenceAsync(PLACE_PRELOAD_LEFT);
                    }
                    else if (cazAuto ==2)
                    {
                        drive.followTrajectorySequenceAsync(PLACE_PRELOAD_MID);
                    }
                    else if (cazAuto == 3)
                    {
                        drive.followTrajectorySequenceAsync(PLACE_PRELOAD_RIGHT);
                    }
                    status = STROBOT.PLACE_PURPLE_PIXEL;
                    break;
                }
                case PLACE_PURPLE_PIXEL:
                {
                    if (!drive.isBusy())
                    {
//                        timerPunerePixel.reset();
                        status = STROBOT.WAIT_FOR_PURPLE_PIXEL;
                    }
                    break;
                }
                case WAIT_FOR_PURPLE_PIXEL:
                {
                    if (timerPunerePixel.seconds()>timePlacePixel)
                    {
                        if (cazAuto == 1)
                        {
                            drive.followTrajectorySequenceAsync(PLACE_SPIKE_LEFT);
                        }
                        else if (cazAuto ==2)
                        {
                            drive.followTrajectorySequenceAsync(PLACE_SPIKE_MID);
                        }
                        else if (cazAuto == 3)
                        {
                            drive.followTrajectorySequenceAsync(PLACE_SPIKE_RIGHT);
                        }
                        timerLift.reset();
                        status = STROBOT.PLACE_SPIKE_BACKDROP;
                    }
                    break;
                }
                case PLACE_SPIKE_BACKDROP:
                {
                    if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT
                            && timerLift.seconds()>delayLift)
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.LOW_AUTO;
                    }
                    if (!drive.isBusy())
                    {
                        scoringController.currentStatus = ScoringController.ScoringStatus.DROP_BOTH_PIXELS;
                        timerPunerePixel.reset();
                        status = STROBOT.GO_TO_STACK_FIRST;
                    }
                    break;
                }
                case GO_TO_STACK_FIRST:
                {
                    if (timerPunerePixel.seconds() > waitTimeBackDrop)
                    {
                        nrCycles++;
                        if(nrCycles>howManyCycles) {
                            drive.followTrajectorySequenceAsync(PARK_ROBOT);
                        }
                        else if (nrCycles == 1)
                        {
                            if (cazAuto == 1)
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_STACK_LEFT);
                            }
                            else if (cazAuto ==2)
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_STACK_MID);
                            }
                            else if (cazAuto ==3)
                            {
                                drive.followTrajectorySequenceAsync(GO_TO_STACK_RIGHT);
                            }
                        }
                        else if(nrCycles==2)
                        {
                            if (nrCycles > howManyCycles)
                            {
                                drive.followTrajectorySequenceAsync(PARK_ROBOT);
                            }
                            else
                            {
                                if(cazAuto==1)
                                    drive.followTrajectorySequenceAsync(GO_TO_STACK_CYCLE2_C1);
                                else if(cazAuto==2)
                                    drive.followTrajectorySequenceAsync(GO_TO_STACK_CYCLE2_C2);
                                else
                                    drive.followTrajectorySequenceAsync(GO_TO_STACK_CYCLE2_C3);
                            }
                        }
//                        else if(nrCycles==3)
//                        {
//                            if (nrCycles > howManyCycles)
//                            {
//                                drive.followTrajectorySequenceAsync(PARK_ROBOT);
//                            }
//                            else
//                            {
//                                // drive.followTrajectorySequenceAsync(GO_TO_STACK_CYCLE3);
//                            }
//                        }

                        timerLift.reset();
                        status = STROBOT.RETRACT_LIFT;
                    }
                    break;
                }
                case RETRACT_LIFT:
                {
                    if (timerLift.seconds()> 0.45 && (liftMotorController.currentStatus != LiftMotorController.LiftStatus.GOING_DOWN && liftMotorController.currentStatus != LiftMotorController.LiftStatus.INIT))
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.GOING_DOWN;
                        status = STROBOT.GO_COLLECT_PIXELS;
                        timerSlides.reset();
                    }
//                    else if(liftMotorController.currentPosition>=-120 && liftMotorController.currentStatus == LiftMotorController.LiftStatus.GOING_DOWN && liftMotorController.currentStatus != LiftMotorController.LiftStatus.INIT)
//                    {
//                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
//                    }

                    if (!drive.isBusy())
                    {
//                        status = STROBOT.GO_COLLECT_PIXELS;
//                        timerSlides.reset();
//                        if(nrCycles == 1) {
//                            if(cazAuto==1)
//                                drive.followTrajectorySequenceAsync(GO_COLLECT_STACK_CYCLE1_C1);
//                            else if(cazAuto==2)
//                                drive.followTrajectorySequenceAsync(GO_COLLECT_STACK_CYCLE1_C2);
//                            else
//                                drive.followTrajectorySequenceAsync(GO_COLLECT_STACK_CYCLE1_C3);
//                        }
//                        else if(nrCycles == 2){
//                            if(cazAuto==1)
//                                drive.followTrajectorySequenceAsync(GO_COLLECT_STACK_CYCLE2_C1);
//                            else if(cazAuto==2)
//                                drive.followTrajectorySequenceAsync(GO_COLLECT_STACK_CYCLE2_C2);
//                            else
//                                drive.followTrajectorySequenceAsync(GO_COLLECT_STACK_CYCLE2_C3);
//                        }
//                        else if(nrCycles == 3){
//                            //drive.followTrajectorySequenceAsync(GO_COLLECT_STACK_CYCLE3);
//                        }
                    }
                    break;
                }
                case GO_COLLECT_PIXELS:
                {
                    if (timerSlides.seconds()>timeOpenSlides && extenderController.currentStatus != ExtenderController.ExtenderStatus.CLOSE && drive.getPoseEstimate().getX()< -30)
                    {
                        extenderController.currentStatus = ExtenderController.ExtenderStatus.CLOSE;
                        if(collectForbarController.currentStatus != CollectForbarController.CollectStatus.COLLECT_DRIVE) {
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_5;
                            if(nrCycles==1){
                                if(DID_FAILSAFE == true) collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_LOW;
                            }
                            if (nrCycles == 2) {
                                collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_LOW;
                                if(DID_FAILSAFE == true) collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                            }
                            if (nrCycles == 3)
                                collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                        }
                        intakeController.currentStatus = IntakeController.IntakeStatus.STACK;
                        timeoutColectare.reset();
                    }
                    if (robot.beamFront.getState() == false && robot.beamBack.getState() == false)
                    {
                        transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                        if(flag)
                            timerReverseScurt.reset();
                        flag = false;
                        status = STROBOT.LEAVE_WITH_2_PIXELS;
                    }
                    else if(extenderController.currentStatus == ExtenderController.ExtenderStatus.CLOSE && timeoutColectare.seconds()>4.5 && nrCycles==3)
                    {
                        transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                        status = STROBOT.LEAVE_WITH_2_PIXELS;
                    }
                    else if(robot.beamBack.getState() == false)
                    {
                        if(nrCycles==1){
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_4;
                        }
                        else if(nrCycles==2)
                        {
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                        }
                        if (timeoutColectare.seconds() > timeout_1pixel) {
//                            turretController.currentStatus = TurretController.Pixel2Status.OPEN;
                            // Pun timpul pentru extendo
                            transferController.actualTimeForExtendo = TransferController.timerExtendoToInit;
                            extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                            if(timerReverseScurt.seconds()<time_reverse_scurt)
                            {
                                intakeController.currentStatus= IntakeController.IntakeStatus.REVERSE;
                            }
                            else
                            {
                                intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
                            }
                            transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;

//                            if (nrCycles == 1) {
//                                if(cazAuto==1)
//                                    drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD_CYCLE1_C1);
//                                else if(cazAuto==2)
//                                    drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD_CYCLE1_C2);
//                                else
//                                    drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD_CYCLE1_C2);
//                            }
//
//                            else if (nrCycles == 2) {
//                                if(cazAuto==1)
//                                    drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD_CYCLE2_C1);
//                                else if(cazAuto==2)
//                                    drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD_CYCLE2_C2);
//                                else
//                                    drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD_CYCLE2_C3);
//                            }

//                                else if (nrCycles == 3) {
////                                    if(cazAuto==1)
////                                        drive.followTrajectorySequenceAsync(GO_COLLECT_STACK_CYCLE3_C1);
////                                    else if(cazAuto==2)
////                                        drive.followTrajectorySequenceAsync(GO_COLLECT_STACK_CYCLE3_C2);
////                                    else
////                                        drive.followTrajectorySequenceAsync(GO_COLLECT_STACK_CYCLE3_C3);
//                                }
                            drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD);
                            status = STROBOT.PLACE_STACK_PIXELS_BB;
                            timerLift.reset();
                        }
                    }
                    else if(robot.beamBack.getState() == true && robot.beamFront.getState() == true && extenderController.currentStatus == ExtenderController.ExtenderStatus.CLOSE)
                    {
                        if (timeoutColectare.seconds() > timeout_nopixel) {
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                            extenderController.currentStatus = ExtenderController.ExtenderStatus.FAILSAFE;
                            status = STROBOT.FAILSAFE_NO_PIXELS;
                        }
                    }
                    break;
                }
                case FAILSAFE_NO_PIXELS: {
                    intakeController.currentStatus= IntakeController.IntakeStatus.REVERSE;
                    collectForbarController.currentStatus= CollectForbarController.CollectStatus.PLAY;
                    DID_FAILSAFE = true;
                    if (Math.abs(extenderCurrentPosition - ExtenderController.extenderFailsafe) <= 30)
                    {
                        //extenderController.currentStatus= ExtenderController.ExtenderStatus.FAR;
                        status = STROBOT.GO_COLLECT_PIXELS;
                    }
                    break;
                }
                case LEAVE_WITH_2_PIXELS:
                {
                    //wait(300);
//                    turretController.currentStatus = TurretController.Pixel2Status.OPEN;

                    // Pun timpul pentru extendo
                    transferController.actualTimeForExtendo = TransferController.timerExtendoToInit;
                    extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                    if(timerReverseScurt.seconds()<time_reverse_scurt)
                    {
                        intakeController.currentStatus= IntakeController.IntakeStatus.REVERSE;
                    }
                    else
                    {
                        intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
                    }
                    //  intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE;
                    if (nrCycles == 1) {
                        drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD);
                    }

                    else if (nrCycles == 2) {
                    }

                    else if (nrCycles == 3) {

                    }
                    status = STROBOT.PLACE_STACK_PIXELS_BB;
                    timerLift.reset();
                    break;
                }
                case PLACE_STACK_PIXELS_BB:
                {
                    if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT &&
                            transferController.currentStatus == TransferController.TransferStatus.INIT && timerLift.seconds()>liftStack && drive.getPoseEstimate().getX()>0)
                    {
                        if(cazAuto==1) {
                            liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE1_C1;
                            if (nrCycles == 2) {
                                liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE2_C1;
                            }
                            else if(nrCycles==3){
                                liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE3;
                            }
                        }
                        else {
                            liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE1_C23;
                            if (nrCycles == 2) {
                                liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE2_C23;
                            }
                            else if(nrCycles==3){
                                liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE3;
                            }
                        }
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.liftAngle;
                        timerDrop.reset();
                    }
                    if (timerDrop.seconds()> timeToDrop && (liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE1_C1 || liftMotorController.currentStatus==LiftMotorController.LiftStatus.liftAngle || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE2_C1 || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE1_C23 || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE2_C23 || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE3))
                    {
                        scoringController.currentStatus = ScoringController.ScoringStatus.DROP_BOTH_PIXELS;
                        timerPunerePixel.reset();
                        status = STROBOT.GO_TO_STACK_FIRST;
                    }
                    break;
                }
            }
            if(AutoTimer.seconds()>28)
            {
                extenderController.currentStatus= ExtenderController.ExtenderStatus.INIT;
            }
            if(AutoTimer.seconds()>29)
            {
                liftMotorController.currentStatus = LiftMotorController.LiftStatus.GOING_DOWN;
            }
            if(robot.liftMotor.getCurrentPosition()>=-120 && liftMotorController.currentStatus == LiftMotorController.LiftStatus.GOING_DOWN)
            {
                liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
            }





            cataratController.update();
            avionController.update();
            extenderController.update(extenderCurrentPosition);
            rotateClawController.update();
            forbarOuttakeController.update();
            liftMotorController.update();
            intakeController.update();
            collectForbarController.update();
            tubuleteController.update();
            transferController.update();
            turretController.update();
            parbrizController.update();
            sigurantaOuttakeController.update();
            scoringController.update();
            drive.update();
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
            telemetry.addData("Cycle: ",nrCycles);
            telemetry.addData("Status",status);
            telemetry.addData("AutoTimer", AutoTimer.seconds());
            telemetry.addData("Extender status", extenderController.currentStatus);
            telemetry.addData("lift status", liftMotorController.currentStatus);
            telemetry.addData("lift position", liftMotorController.currentPosition);
            telemetry.addData("BeamFront", robot.beamFront.getState());
            telemetry.addData("BeamBack", robot.beamBack.getState());
            telemetry.update();
        }
    }

}