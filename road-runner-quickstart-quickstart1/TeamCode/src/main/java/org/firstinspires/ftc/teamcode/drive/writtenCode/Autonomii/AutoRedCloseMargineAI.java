package org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.initPosition;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

public class AutoRedCloseMargineAI extends LinearOpMode {

    static Pose2d startPose = new Pose2d(10, -62, Math.toRadians(90));
    public static double forwardRight = 6.75;
    public static  double PRELOAD_RIGHT_X = 16;
    public static  double PRELOAD_RIGHT_Y = -43.5;
    public static  double PRELOAD_ANGLE_RIGHT = 45;

    public static  double PLACE_SPIKE_RIGHT_X = 49;
    public static  double PLACE_SPIKE_RIGHT_Y = -46;
    public static  double ANGLE_SPIKE_RIGHT = 0;

    public static  double PARK_RIGHT_X = 43;
    public static  double PARK_RIGHT_Y = -20;
    public static  double ANGLE_PARK_RIGHT = 0;

    public static  double PRELOAD_MID_X = 10;
    public static  double PRELOAD_MID_Y = -34.1;
    public static  double PRELOAD_ANGLE_MID = 90;

    public static  double PLACE_SPIKE_MID_X = 50;
    public static  double PLACE_SPIKE_MID_Y = -39.7;
    public static  double ANGLE_SPIKE_MID = 0;

    public static  double PARK_MID_X = 43;
    public static  double PARK_MID_Y = -20;
    public static  double ANGLE_PARK_MID = 0;

    public static  double PRELOAD_LEFT_X = 7;
    public static  double PRELOAD_LEFT_Y = -41;
    public static  double PRELOAD_ANGLE_LEFT = 140;

    public static  double PLACE_SPIKE_LEFT_X = 48.5;
    public static  double PLACE_SPIKE_LEFT_Y = -32;
    public static double ANGLE_SPIKE_LEFT = 0;

    public static double PARK_LEFT_X = 43;
    public static double PARK_LEFT_Y = -62;
    public static double ANGLE_PARK_LEFT = 0;


    public static final double GO_TO_STACK_X = 30f;
    public static final double GO_TO_STACK_Y = -56.5f;
    public static final double LINE_TO_STACK_X = -29.5f;
    public static final double LINE_TO_STACK_Y = -56f;
    public static final double GO_TO_BACK_1_X = -55f;
    public static final double GO_TO_BACK_1_Y =-56f;
    public static final double GO_TO_BACK_2_X = 10f;
    public static final double GO_TO_BACK_2_Y =-56f;
    public static final double ANGLE_COLECTARE=-47.5f;
    public static final double ANGLE_COLECTARE_CYCLE2=-46.7f;

    public static final double COLLECT_STACK_X_CYCLE1_C1 = -46f;
    public static final double COLLECT_STACK_Y_CYCLE1_C1 = -53f;
    public static final double COLLECT_STACK_X_CYCLE2_C1 = -46f;
    public static final double COLLECT_STACK_Y_CYCLE2_C1 = -53f;

    public static final double COLLECT_STACK_X_CYCLE1_C2 = -52f;
    public static final double COLLECT_STACK_Y_CYCLE1_C2 = -29.7f;
    public static final double COLLECT_STACK_X_CYCLE2_C2 = -50.5f;
    public static final double COLLECT_STACK_Y_CYCLE2_C2 = -29.7f;


    public static final double COLLECT_STACK_X_CYCLE1_C3 = -50f;
    public static final double COLLECT_STACK_Y_CYCLE1_C3 = -30.5f;
    public static final double COLLECT_STACK_X_CYCLE2_C3 = -50f;
    public static final double COLLECT_STACK_Y_CYCLE2_C3 = -30.5f;






    public static final double PLACE_BB_LLH1_X = 20;
    public static final double PLACE_BB_LLH1_Y = -11;
    public static final int ANGLE_BB_LLH1 = 0;
    public static final double PLACE_BB_LLH2_X = 49f;
    public static final double PLACE_BB_LLH2_Y = -39.5f;
    public static final double ANGLE_BB_LLH2 = 0f;


    public static float timeout_1pixel = 2.5f;
    public static float timeout_nopixel = 2.5f;
    public static float timeout_nopixel_C23=5f;
    public static float time_reverse_scurt =0f;
    public static float timeToDrop = 1f;

    public static int CAZ_BUN = 0;
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
    public static double waitTimeBackDrop = 0.5;
    public static double timeOpenSlides = 4.6;
    public static double timeOpenSlidesC3= 5f; //4.5

    public static double liftStack= 3.5;
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
    public static int cazAuto = 1;


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
        cameraRecognition = new CaseDetectionPipeline(hardwareMap,telemetry,"red");
        cameraRecognition.initCamera();
        cameraRecognition.start(1);


        Pose2d startPose = new Pose2d(10, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        //  TrajectoryVelocityConstraint VELLLH = getVelocityConstraint(40, 5, 12.05);
        double nrCycles = 0;
        double howManyCycles = 2;
        TrajectorySequence PLACE_PRELOAD_LEFT = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PRELOAD_LEFT_X, PRELOAD_LEFT_Y,Math.toRadians(PRELOAD_ANGLE_LEFT)))
                .back(forwardRight)
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
                .back(10)
                .build();
        TrajectorySequence PLACE_SPIKE_MID = drive.trajectorySequenceBuilder(PLACE_PRELOAD_MID.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_MID_X, PLACE_SPIKE_MID_Y,Math.toRadians(ANGLE_SPIKE_MID)))
                .build();


        TrajectorySequence PLACE_PRELOAD_RIGHT = drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(VELLLH)
                .lineToLinearHeading(new Pose2d(PRELOAD_RIGHT_X, PRELOAD_RIGHT_Y,Math.toRadians(PRELOAD_ANGLE_RIGHT)))
                .lineTo(new Vector2d(14,-53))
                .build();

        TrajectorySequence PLACE_SPIKE_RIGHT = drive.trajectorySequenceBuilder(PLACE_PRELOAD_RIGHT.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_RIGHT_X, PLACE_SPIKE_RIGHT_Y,Math.toRadians(ANGLE_SPIKE_RIGHT)))
                .build();


        TrajectorySequence GO_TO_STACK_LEFT = drive.trajectorySequenceBuilder(PLACE_SPIKE_LEFT.end())
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y),Math.toRadians(180))
                .lineTo(new Vector2d(GO_TO_STACK_X,GO_TO_STACK_Y))
                .lineTo(new Vector2d(LINE_TO_STACK_X, LINE_TO_STACK_Y))
//                .lineToLinearHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .lineToSplineHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C1, COLLECT_STACK_Y_CYCLE1_C1,Math.toRadians(ANGLE_COLECTARE)))
                // .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_2, COLLECT_STACK_Y_CYCLE1_C2))
//                .back(9)
                .build();

        TrajectorySequence GO_TO_STACK_MID = drive.trajectorySequenceBuilder(PLACE_SPIKE_MID.end())
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y),Math.toRadians(180))
                .lineTo(new Vector2d(GO_TO_STACK_X,GO_TO_STACK_Y))
                .lineTo(new Vector2d(LINE_TO_STACK_X, LINE_TO_STACK_Y))
//                .lineToLinearHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .splineToConstantHeading(new Vector2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2),Math.toRadians(180))
                // .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_2, COLLECT_STACK_Y_CYCLE1_C2))
                .back(9)
                .build();


        TrajectorySequence GO_TO_STACK_RIGHT = drive.trajectorySequenceBuilder(PLACE_SPIKE_RIGHT.end())
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y),Math.toRadians(180))
                .lineTo(new Vector2d(GO_TO_STACK_X,GO_TO_STACK_Y))
                .lineTo(new Vector2d(LINE_TO_STACK_X, LINE_TO_STACK_Y))
//                .lineToLinearHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .splineToConstantHeading(new Vector2d(COLLECT_STACK_X_CYCLE1_C3, COLLECT_STACK_Y_CYCLE1_C3),Math.toRadians(180))
                // .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_2, COLLECT_STACK_Y_CYCLE1_C2))
                .back(9)
                .build();

        TrajectorySequence GO_PLACE_ON_BACKBOARD = drive.trajectorySequenceBuilder(new Pose2d(COLLECT_STACK_X_CYCLE1_C3, COLLECT_STACK_Y_CYCLE1_C3,Math.toRadians(ANGLE_COLECTARE)))
                .lineToLinearHeading(new Pose2d(GO_TO_BACK_1_X,GO_TO_BACK_1_Y,Math.toRadians(0)))
//                .lineTo(new Vector2d(LINE_TO_STACK_X,LINE_TO_STACK_Y))
                .lineTo(new Vector2d(GO_TO_BACK_2_X,GO_TO_BACK_2_Y))
//                .lineToLinearHeading(new Pose2d(PLACE_BB_LLH2_X,PLACE_BB_LLH2_Y,Math.toRadians(ANGLE_BB_LLH2)))
//                .turn(Math.toRadians(-40))
                .splineToConstantHeading(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y),Math.toRadians(0))
                // .lineTo(new Vector2d(PLACE_BB_LLH2_X, PLACE_BB_LLH2_Y))
                .build(); // te duce la backboard

        TrajectorySequence GO_TO_STACK_CYCLE2_C1 = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD.end())
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y),Math.toRadians(180))
                .lineTo(new Vector2d(GO_TO_STACK_X,GO_TO_STACK_Y))
                .lineTo(new Vector2d(LINE_TO_STACK_X, LINE_TO_STACK_Y))
//                .lineToLinearHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .lineToSplineHeading(new Pose2d(COLLECT_STACK_X_CYCLE2_C1, COLLECT_STACK_Y_CYCLE2_C1,Math.toRadians(ANGLE_COLECTARE_CYCLE2)))
                // .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_2, COLLECT_STACK_Y_CYCLE1_C2))
//                .back(9)
                .build();

        TrajectorySequence GO_TO_STACK_CYCLE2_C2 = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD.end())
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y),Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(GO_TO_STACK_X,GO_TO_STACK_Y,Math.toRadians(0)))
                .lineTo(new Vector2d(LINE_TO_STACK_X, LINE_TO_STACK_Y))
//                .lineToLinearHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .splineToConstantHeading(new Vector2d(COLLECT_STACK_X_CYCLE2_C2, COLLECT_STACK_Y_CYCLE2_C2),Math.toRadians(180))
                // .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_2, COLLECT_STACK_Y_CYCLE1_C2))
                .back(10)
                .build();
        TrajectorySequence GO_TO_STACK_CYCLE2_C3 = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD.end())
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(GO_TO_STACK_X, GO_TO_STACK_Y),Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(GO_TO_STACK_X,GO_TO_STACK_Y,Math.toRadians(0)))
                .lineTo(new Vector2d(LINE_TO_STACK_X, LINE_TO_STACK_Y))
//                .lineToLinearHeading(new Pose2d(COLLECT_STACK_X_CYCLE1_C2, COLLECT_STACK_Y_CYCLE1_C2,Math.toRadians(ANGLE_COLECTARE)))
                .splineToConstantHeading(new Vector2d(COLLECT_STACK_X_CYCLE2_C3, COLLECT_STACK_Y_CYCLE2_C3),Math.toRadians(180))
                // .lineToConstantHeading(new Vector2d(COLLECT_STACK_X_2, COLLECT_STACK_Y_CYCLE1_C2))
                .back(10)
                .build();


        TrajectorySequence PARK_ROBOT = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD.end())
                .back(10)
                .lineToLinearHeading(new Pose2d(PARK_RIGHT_X, PARK_RIGHT_Y,Math.toRadians(ANGLE_PARK_RIGHT)))
                .build();

        while (!isStarted()&&!isStopRequested())
        {
            if(CAZ_BUN == 0) {
                int c = cameraRecognition.getCase();
                telemetry.addData("detected", c);
                cazAuto = c;
                telemetry.addLine("Init Complete");
                telemetry.update();
                sleep(50);
            }
            else
            {
                cazAuto = CAZ_BUN;
                telemetry.addData("detected", cazAuto);
                telemetry.addLine("Init Complete");
                telemetry.update();
            }
        }
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
                                else if(cazAuto==3)
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
                    if(rotateClawController.currentStatus== RotateClawController.RotateStatus.HORIZONTAL) rotateClawController.currentStatus= RotateClawController.RotateStatus.VERTICAL;
                    parbrizController.currentStatus= ParbrizController.ParbrizStatus.CLOSED;
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
                    flag=true;
                    break;
                }
                case GO_COLLECT_PIXELS: {
                    intakeController.currentStatus = IntakeController.IntakeStatus.STACK;
                    if (flag == true){
                        collectForbarController.currentStatus = CollectForbarController.CollectStatus.PLAY;
//                        timeoutColectare.reset();
                        flag = false;
                    }
                    if (timerSlides.seconds()>timeOpenSlides && drive.getPoseEstimate().getX()< -30 && cazAuto!=1)
                    {
                        if(collectForbarController.currentStatus != CollectForbarController.CollectStatus.COLLECT_DRIVE) {
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.AI;
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
                        timeoutColectare.reset();

                    }
                    else if(cazAuto==1)
                    {
                        if (timerSlides.seconds()>timeOpenSlidesC3 && extenderController.currentStatus != ExtenderController.ExtenderStatus.CLOSE_AUTO_RED && drive.getPoseEstimate().getX()< -30)
                        {
                            extenderController.currentStatus = ExtenderController.ExtenderStatus.CLOSE_AUTO_RED;
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.AI;
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
                    }
                    if (robot.beamFront.getState() == false && robot.beamBack.getState() == false)
                    {
                        transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                        if(flag)
                            timerReverseScurt.reset();
                        flag = false;
                        status = STROBOT.LEAVE_WITH_2_PIXELS;
                    }
                    else if(extenderController.currentStatus == ExtenderController.ExtenderStatus.CLOSE_AUTO_RED && timeoutColectare.seconds()>4.5 && nrCycles==3)
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
                    else if(robot.beamBack.getState() == true && robot.beamFront.getState() == true && extenderController.currentStatus == ExtenderController.ExtenderStatus.CLOSE_AUTO_RED)
                    {
                        if (timeoutColectare.seconds() > timeout_nopixel) {
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                            extenderController.currentStatus = ExtenderController.ExtenderStatus.FAILSAFE;
                            status = STROBOT.FAILSAFE_NO_PIXELS;
                        }
                    }
//                    else if(robot.beamBack.getState() == true && robot.beamFront.getState() == true)
//                    {
//                        if (timeoutColectare.seconds() > timeout_nopixel_C23) {
//                            status = STROBOT.FAILSAFE_NO_PIXELS;
//                        }
//                    }
                    break;
                }
                case FAILSAFE_NO_PIXELS: {
                    if(cazAuto==1){
                        intakeController.currentStatus= IntakeController.IntakeStatus.REVERSE;
                        collectForbarController.currentStatus= CollectForbarController.CollectStatus.PLAY;
                        DID_FAILSAFE = true;
                        if (Math.abs(extenderCurrentPosition - ExtenderController.extenderFailsafe) <= 30)
                        {
                            //extenderController.currentStatus= ExtenderController.ExtenderStatus.FAR;
                            status = STROBOT.GO_COLLECT_PIXELS;
                        }
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
                        drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD);
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
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_CYCLE1_C1;

                        timerDrop.reset();
                    }
                    if(liftMotorController.currentPosition<-120) rotateClawController.currentStatus = RotateClawController.RotateStatus.HORIZONTAL;
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
                rotateClawController.currentStatus = RotateClawController.RotateStatus.VERTICAL;
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
            telemetry.addData("timeoutcolectare", timeoutColectare.seconds());
            telemetry.addData("4bar status", collectForbarController.currentStatus);
            telemetry.update();
        }
    }

}