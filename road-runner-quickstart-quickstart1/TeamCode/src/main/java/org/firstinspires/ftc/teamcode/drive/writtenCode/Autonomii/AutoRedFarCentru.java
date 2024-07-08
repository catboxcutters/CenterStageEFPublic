package org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii;

import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.initPosition;

import android.view.inputmethod.ExtractedText;

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
import org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.OuttakeSlidesController;
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

public class AutoRedFarCentru extends LinearOpMode {


    public static double forwardRight = 6.75;

    public static  double PRELOAD_RIGHT_X = -32.5;
    public static  double PRELOAD_RIGHT_Y = -36;
    public static  double PRELOAD_ANGLE_RIGHT = 39.5;

    public static  double PLACE_SPIKE_RIGHT_X = 49;
    public static  double PLACE_SPIKE_RIGHT_Y = -40;
    public static double ANGLE_SPIKE_RIGHT = -30;

    public static  double PRELOAD_MID_X = -36;
    public static  double PRELOAD_MID_Y = -35;
    public static  double PRELOAD_ANGLE_MID = 42;

    public static  double PLACE_SPIKE_MID_X = 48;
    public static  double PLACE_SPIKE_MID_Y = -42.5;
    public static  double ANGLE_SPIKE_MID = -35;


    public static  double PRELOAD_LEFT_X = -41.5;
    public static  double PRELOAD_LEFT_Y = -38;
    public static  double PRELOAD_ANGLE_LEFT = 145;

    public static  double PLACE_SPIKE_LEFT_X = 49;
    public static  double PLACE_SPIKE_LEFT_Y = -27.5;
    public static double ANGLE_SPIKE_LEFT = -30;

    public static double PARK_RIGHT_X = 43;
    public static double PARK_RIGHT_Y = -10;
    public static double ANGLE_PARK_RIGHT = 0;

    public static final double FIRST_PIXEL_X = -56;
    public static final double FIRST_PIXEL_Y = -10;

    public static double FIRST_PIXEL_X_C1 =-53.5;
    public static double FIRST_PIXEL_Y_C1 =-21;
    public static double FIRST_PIXEL_X_C2 =-47.6;
    public static double FIRST_PIXEL_Y_C2 =-20.5; //19.8
    public static double FIRST_PIXEL_X_C3 =-32;
    public static double FIRST_PIXEL_Y_C3 =-30;

    public static double GO_TO_BB_PRELOAD_X =-40;
    public static double GO_TO_BB_PRELOAD_Y=-12;
    public static double GO_TO_BB_PRELOAD_X_LEFT =-55;
    public static double GO_TO_BB_PRELOAD_Y_LEFT =-12;
    public static double GO_TO_BB_PRELOAD_X_MID =-48;
    public static double GO_TO_BB_PRELOAD_Y_MID =-12;

    public static double GO_TO_STACK_X_C3 = 10f;
    public static double GO_TO_STACK_Y_C3 = -12f;
    public static double COLLECT_STACK_X_C3 = -23.5f;
    public static double COLLECT_STACK_Y_C3 = -12f;

    public static double GO_TO_STACK_X_C2 = 10f;
    public static double GO_TO_STACK_Y_C2 = -12f;
    public static double COLLECT_STACK_X_C2 = -25f;
    public static double COLLECT_STACK_Y_C2 = -12f;

    public static final double GO_TO_STACK_X_MID = 11f;
    public static final double GO_TO_STACK_Y_MID = -12f;
    public static final double COLLECT_STACK_X_MID = -24.5f;
    public static final double COLLECT_STACK_Y_MID = -12f;

    public static final double GO_TO_STACK_X_C1 = 11.5f;
    public static final double GO_TO_STACK_Y_C1 = -12f;
    public static final double COLLECT_STACK_X_C1 = -25f;
    public static final double COLLECT_STACK_Y_C1 = -12f;

    public static final double COLLECT_STACK_ANGLE=0f;

    public static final double PLACE_BB_LLH1_X_CYCLES = 15;
    public static final double PLACE_BB_LLH1_Y_CYCLES = -14f;
    public static final double PLACE_BB_LLH2_X_CYCLES = 48.5;
    public static final double PLACE_BB_LLH2_Y_CYCLES = -27.7;
    public static final double PLACE_BB_LLH2_ANGLE_CYCLES = -30;




    public static final double PLACE_BB_LLH1_X = 10;
    public static final double PLACE_BB_LLH1_Y = -12;


    public static double PLACE_BB_LLH1_X_3 = 10;
    public static double PLACE_BB_LLH1_Y_3 = -12;

    public static final double PLACE_BB_LLH2_X = 44f;
    public static final double PLACE_BB_LLH2_Y = -22f;
    public static final double ANGLE_BB_LLH2 = -40f;

    public static final double PLACE_BB_LLH1_X_PRELOAD = 15;
    public static final double PLACE_BB_LLH1_Y_PRELOAD = -14;


    public static int timeout_1pixel = 3;
    public static float timeout_nopixel = 2f;
    public static float time_reverse_scurt =0f;
    public static float timeToDrop = 1f;
    public static float timeToFirst;
    public static float leave_first = 4f;
    public static int CAZ_BUN = 2;
    boolean flag2=false;
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
        TAKE_ONE_PIXEL,
        PLACE_SPIKE_BACKDROP,
        PARK,
        RETRACT_LIFT,
        GO_COLLECT_PIXELS,
        LEAVE_WITH_2_PIXELS,
        PLACE_STACK_PIXELS_BB,
        GO_TO_STACK_FIRST,
        END_AUTO,
        FAILSAFE_NO_PIXELS,
        COLLECT_FAILSAFE,
        PLACE_FIRST_PIXEL,

    }
    public static double timePlacePixel = 0.3;
    public static double delayLift = 2;
    public static double waitTimeBackDrop1 = 0.8;
    public static double waitTimeBackDrop = 0.6;
    public static double timeOpenSlides = 2;
    public static double timeRaiseLiftFirst = 0.3;
    public static double timePlacePreloadPurple;
    public static float timetostart=0.5f;

    public static double liftStack= 1.5;
    public static boolean flag =true;
    boolean DID_FAILSAFE = false;
    boolean DID_FAILSAFE_TWICE = false;
    ElapsedTime timerPunerePixel = new ElapsedTime();
    ElapsedTime timerLift = new ElapsedTime();
    ElapsedTime timerSlides = new ElapsedTime();
    ElapsedTime timerDrop = new ElapsedTime();
    ElapsedTime timeoutColectare = new ElapsedTime();
    ElapsedTime timerReverseScurt = new ElapsedTime();
    ElapsedTime AutoTimer = new ElapsedTime();
    ElapsedTime TimerFirst = new ElapsedTime();
    CaseDetectionPipeline cameraRecognition;
    public static double cazAuto = 1;


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
        OuttakeSlidesController outtakeSlidesController = new OuttakeSlidesController(robot);
        robot.forbarCutieIntake.setPosition(initPosition);
        robot.clawRotate.setPosition(RotateClawController.verticalServoPosition);
//        turretController.currentStatus = TurretController.Pixel2Status.OPEN;

        TransferController transferController = new TransferController(
                intakeController,tubuleteController,sigurantaOuttakeController, extenderController,robot);
        ScoringController scoringController = new ScoringController(turretController, sigurantaOuttakeController, parbrizController, rotateClawController);

        cataratController.update();
        avionController.update();
        extenderController.update(ExtenderController.extenderInit);
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
        outtakeSlidesController.update();
        cameraRecognition = new CaseDetectionPipeline(hardwareMap,telemetry,"red");
        cameraRecognition.initCamera();
        cameraRecognition.start(1);


        Pose2d startPose = new Pose2d(-38.5, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        //  TrajectoryVelocityConstraint VELLLH = getVelocityConstraint(40, 5, 12.05);
        double nrCycles = 0;
        double howManyCycles = 3;
        TrajectorySequence PLACE_PRELOAD_LEFT = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PRELOAD_LEFT_X, PRELOAD_RIGHT_Y,Math.toRadians(PRELOAD_ANGLE_LEFT)))
                .build();

        TrajectorySequence PLACE_PRELOAD_MID = drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(VELLLH)
                .lineToLinearHeading(new Pose2d(PRELOAD_MID_X, PRELOAD_MID_Y,Math.toRadians(PRELOAD_ANGLE_MID)))
                .build();


        TrajectorySequence PLACE_PRELOAD_RIGHT = drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(VELLLH)
                .lineToSplineHeading(new Pose2d(PRELOAD_RIGHT_X, PRELOAD_RIGHT_Y,Math.toRadians(PRELOAD_ANGLE_RIGHT)))
                .build();

        TrajectorySequence FIRST_PIXEL_C1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(FIRST_PIXEL_X_C1,FIRST_PIXEL_Y_C1,Math.toRadians(-48)))
                .build();
        TrajectorySequence FIRST_PIXEL_C2 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(FIRST_PIXEL_X_C2,FIRST_PIXEL_Y_C2,Math.toRadians(-30)))
                .build();
        TrajectorySequence FIRST_PIXEL_C3 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(FIRST_PIXEL_X_C3,FIRST_PIXEL_Y_C3,Math.toRadians(-36)))
//                .back(5)+
                .build();

        TrajectorySequence PLACE_SPIKE_LEFT = drive.trajectorySequenceBuilder(FIRST_PIXEL_C1.end())
                .lineToLinearHeading(new Pose2d(GO_TO_BB_PRELOAD_X_LEFT,GO_TO_BB_PRELOAD_Y_LEFT, Math.toRadians(0)))
                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y))
                .splineTo(new Vector2d(PLACE_SPIKE_LEFT_X,PLACE_SPIKE_LEFT_Y),Math.toRadians(ANGLE_SPIKE_LEFT))
                .build();
        TrajectorySequence PLACE_SPIKE_MID = drive.trajectorySequenceBuilder(FIRST_PIXEL_C2.end())
                .lineToLinearHeading(new Pose2d(GO_TO_BB_PRELOAD_X_MID,GO_TO_BB_PRELOAD_Y_MID, Math.toRadians(0)))
//                .setReversed(true)
                .lineTo(new Vector2d(PLACE_BB_LLH1_X, PLACE_BB_LLH1_Y))
                .splineToConstantHeading(new Vector2d(PLACE_SPIKE_MID_X,PLACE_SPIKE_MID_Y),Math.toRadians(0))
                .build();
        TrajectorySequence PLACE_SPIKE_RIGHT = drive.trajectorySequenceBuilder(FIRST_PIXEL_C3.end())
                .lineToLinearHeading(new Pose2d(GO_TO_BB_PRELOAD_X,GO_TO_BB_PRELOAD_Y, Math.toRadians(0)))
                .lineTo(new Vector2d(PLACE_BB_LLH1_X_3, PLACE_BB_LLH1_Y_3))
                .splineTo(new Vector2d(PLACE_SPIKE_RIGHT_X,PLACE_SPIKE_RIGHT_Y),Math.toRadians(ANGLE_SPIKE_RIGHT))
                .build();

        TrajectorySequence GO_TO_STACK_LEFT = drive.trajectorySequenceBuilder(PLACE_SPIKE_LEFT.end())
                .setReversed(true)
                .splineTo(new Vector2d(GO_TO_STACK_X_C1, GO_TO_STACK_Y_C1),Math.toRadians(180))
                .lineTo(new Vector2d(COLLECT_STACK_X_C1, COLLECT_STACK_Y_C1))
                .build();

        TrajectorySequence GO_TO_STACK_MID = drive.trajectorySequenceBuilder(PLACE_SPIKE_MID.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(GO_TO_STACK_X_MID, GO_TO_STACK_Y_MID),Math.toRadians(180))
                .lineTo(new Vector2d(COLLECT_STACK_X_MID, COLLECT_STACK_Y_MID))
                .build();

        TrajectorySequence GO_TO_STACK_RIGHT = drive.trajectorySequenceBuilder(PLACE_SPIKE_RIGHT.end())
                .setReversed(true)
                .splineTo(new Vector2d(GO_TO_STACK_X_C3, GO_TO_STACK_Y_C3),Math.toRadians(180))
                .lineTo(new Vector2d(COLLECT_STACK_X_C3, COLLECT_STACK_Y_C3))
                .build();



        TrajectorySequence GO_PLACE_ON_BACKBOARD = drive.trajectorySequenceBuilder(new Pose2d(COLLECT_STACK_X_C1, COLLECT_STACK_Y_C1,Math.toRadians(COLLECT_STACK_ANGLE)))
                .lineTo(new Vector2d(PLACE_BB_LLH1_X_CYCLES,PLACE_BB_LLH1_Y_CYCLES))
                .splineTo(new Vector2d(PLACE_BB_LLH2_X_CYCLES,PLACE_BB_LLH2_Y_CYCLES),Math.toRadians(PLACE_BB_LLH2_ANGLE_CYCLES))
                .build();

        TrajectorySequence GO_TO_STACK_C1 = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD.end())
                .setReversed(true)
                .splineTo(new Vector2d(GO_TO_STACK_X_C1, GO_TO_STACK_Y_C1),Math.toRadians(180))
                .lineTo(new Vector2d(COLLECT_STACK_X_C1, COLLECT_STACK_Y_C1))
                .build();
        TrajectorySequence GO_TO_STACK_CYCLE3_C1 = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD.end())
                .setReversed(true)
                .splineTo(new Vector2d(GO_TO_STACK_X_C1,GO_TO_STACK_Y_C1),Math.toRadians(180))
                .lineTo(new Vector2d(-10,COLLECT_STACK_Y_C1))
                .splineTo(new Vector2d(-30,-15),Math.toRadians(197))
                .build();
        TrajectorySequence GO_PLACE_ON_BACKBOARD_CYCLE3 = drive.trajectorySequenceBuilder(GO_TO_STACK_CYCLE3_C1.end())
                .setReversed(false)
                .splineTo(new Vector2d(-10,COLLECT_STACK_Y_C1),Math.toRadians(0))
                .lineTo(new Vector2d(PLACE_BB_LLH1_X_CYCLES,-12))
                .splineTo(new Vector2d(PLACE_BB_LLH2_X_CYCLES,PLACE_BB_LLH2_Y_CYCLES),Math.toRadians(PLACE_BB_LLH2_ANGLE_CYCLES))
                .build();

        TrajectorySequence GO_TO_STACK_C2 = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD.end())
                .setReversed(true)
                .splineTo(new Vector2d(GO_TO_STACK_X_C2, GO_TO_STACK_Y_C2),Math.toRadians(180))
                .lineTo(new Vector2d(COLLECT_STACK_X_C2, COLLECT_STACK_Y_C2))
                .build();
        TrajectorySequence GO_TO_STACK_C3 = drive.trajectorySequenceBuilder(GO_PLACE_ON_BACKBOARD.end())
                .setReversed(true)
                .splineTo(new Vector2d(GO_TO_STACK_X_C3, GO_TO_STACK_Y_C3),Math.toRadians(180))
                .lineTo(new Vector2d(COLLECT_STACK_X_C3, COLLECT_STACK_Y_C3))
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
            int extenderCurrentPosition = robot.leftExtension.getCurrentPosition();
            switch (status)
            {
                case START:
                {
                    if(flag2==false) {
                        AutoTimer.reset();
                        flag2=true;
                    }
                        status = STROBOT.PLACE_PURPLE_PIXEL;
                    break;
                }
                case PLACE_PURPLE_PIXEL:
                {
                    if (!drive.isBusy())
                    {
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
                            drive.followTrajectorySequenceAsync(FIRST_PIXEL_C1);
                        }
                        else if (cazAuto ==2)
                        {
                            drive.followTrajectorySequenceAsync(FIRST_PIXEL_C2);
                        }
                        else if (cazAuto == 3)
                        {
                            drive.followTrajectorySequenceAsync(FIRST_PIXEL_C3);
                        }
                        timerLift.reset();
                        timerSlides.reset();
                        TimerFirst.reset();
                        status = STROBOT.TAKE_ONE_PIXEL;
                    }
                    flag=true;
                    break;
                }
                case TAKE_ONE_PIXEL:
                {
                    if(flag==true) {
                        collectForbarController.currentStatus = CollectForbarController.CollectStatus.PLAY;
                        flag=false;
                    }
                    if(TimerFirst.seconds()>1.65) {
                        if (cazAuto == 1) {
                            extenderController.currentStatus= ExtenderController.ExtenderStatus.CLOSE_AUTO_C1;
                            timePlacePreloadPurple = 2.5f;
                            timeToFirst=2.7f;
                        }
                        else if(cazAuto == 2){
                            extenderController.currentStatus= ExtenderController.ExtenderStatus.CLOSE_AUTO_C2;
                            timePlacePreloadPurple = 2.5f;
                            timeToFirst=2.7f;
                        }
                        else if(cazAuto==3){
                            extenderController.currentStatus= ExtenderController.ExtenderStatus.CLOSE_AUTO_C3;
                            timePlacePreloadPurple = 2.5f;
                            timeToFirst=2.3f;
                        }
                        if(TimerFirst.seconds()>timeToFirst && collectForbarController.currentStatus != CollectForbarController.CollectStatus.ONE_PIXEL_FAILSAFE) {
                            if (cazAuto==1) {
                                collectForbarController.currentStatus = CollectForbarController.CollectStatus.ONE_PIXEL;
                            }
                            else if(cazAuto==2){
                                collectForbarController.currentStatus = CollectForbarController.CollectStatus.ONE_PIXEL_2;
                            }
                            else {
                                collectForbarController.currentStatus = CollectForbarController.CollectStatus.ONE_PIXEL_3;
                            }
                            intakeController.currentStatus = IntakeController.IntakeStatus.STACK;
                        }
                        timeoutColectare.reset();
//                        if(TimerFirst.seconds()>3) collectForbarController.currentStatus= CollectForbarController.CollectStatus.ONE_PIXEL_FAILSAFE;

                        //forbarOuttakeController.currentStatus= ForbarOuttakeController.ForbarStatus.PRELOAD;
                        if(TimerFirst.seconds()>timePlacePreloadPurple)
                        {
                            if (cazAuto != 3)
                            {
                                outtakeSlidesController.currentStatus= OuttakeSlidesController.ExtensionStatus.PRELOAD12;
                            }
                            else
                            {
                                outtakeSlidesController.currentStatus= OuttakeSlidesController.ExtensionStatus.PRELOAD3;
                            }
                            //scoringController.currentStatus= ScoringController.ScoringStatus.DROP_BOTH_PIXELS;
                        }
                        if(robot.beamBack.getState() == false && robot.beamFront.getState()==false || TimerFirst.seconds()>leave_first) {
                            // Pun timpul pentru extendo
                            transferController.actualTimeForExtendo = TransferController.timerExtendoToInit;
                            intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
                            transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;
                            extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                            outtakeSlidesController.currentStatus= OuttakeSlidesController.ExtensionStatus.COMPACT;
                            forbarOuttakeController.currentStatus= ForbarOuttakeController.ForbarStatus.GET_COLLECTED_PIXELS;
                            if (cazAuto == 1) drive.followTrajectorySequenceAsync(PLACE_SPIKE_LEFT);
                            else if (cazAuto == 2) drive.followTrajectorySequenceAsync(PLACE_SPIKE_MID);
                            else if (cazAuto == 3)
                                drive.followTrajectorySequenceAsync(PLACE_SPIKE_RIGHT);
                            status = STROBOT.PLACE_SPIKE_BACKDROP;
                            timerLift.reset();
                        }
                    }
                    break;
                }
                case PLACE_SPIKE_BACKDROP:
                {
                    if (liftMotorController.currentStatus == LiftMotorController.LiftStatus.INIT
                            && timerLift.seconds()>delayLift && drive.getPoseEstimate().getX()>5)
                        {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.LOW_AUTO;
                        outtakeSlidesController.currentStatus = OuttakeSlidesController.ExtensionStatus.FAR;
                        turretController.currentStatus= TurretController.TurretStatus.RUNTO;
                        rotateClawController.currentStatus = RotateClawController.RotateStatus.RUNTO;
                        timerPunerePixel.reset();
                        }
                    if(LiftMotorController.liftMotor.getCurrentPosition()<-50) {
                        if (cazAuto == 1)
                        {
                            turretController.targetposition = turretController.initPosition + (0.003 * (0 - ANGLE_SPIKE_LEFT));
                            rotateClawController.targetposition = rotateClawController.verticalServoPosition - (0.0010 * (0 - ANGLE_SPIKE_LEFT));
                        }
                        else if (cazAuto == 3)
                        {
                            turretController.targetposition = turretController.initPosition + (0.003 * (0 - ANGLE_SPIKE_RIGHT));
                            rotateClawController.targetposition = rotateClawController.verticalServoPosition - (0.0010 * (0 - ANGLE_SPIKE_RIGHT));

                        }
                    }
                    double timetodropPreload = 1.6;
                    if(cazAuto==2 || cazAuto == 3) timetodropPreload=2;
                    if(timerPunerePixel.seconds()> timetodropPreload && liftMotorController.currentStatus==LiftMotorController.LiftStatus.LOW_AUTO)
                    {
                        // De ce o mai deschizi inca o data daca e deschisa de linia de mai jos?
                        sigurantaOuttakeController.currentStatus= SigurantaOuttakeController.SigurantaOuttakeStatus.OPEN;
                        scoringController.currentStatus= ScoringController.ScoringStatus.DROP_BOTH_PIXELS;
                        timerPunerePixel.reset();
                        status = STROBOT.PLACE_FIRST_PIXEL;
                    }
                    break;
                }
                case PLACE_FIRST_PIXEL:
                {
                    if(timerPunerePixel.seconds() > timeRaiseLiftFirst)
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.AUTO_RAISELIFT;
                        status = STROBOT.GO_TO_STACK_FIRST;
                    }
                    break;
                }
                case GO_TO_STACK_FIRST:
                {
                    if(nrCycles==0) waitTimeBackDrop=waitTimeBackDrop1;
                    else waitTimeBackDrop= 0.6;
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
                                if(cazAuto==1) {
                                    drive.followTrajectorySequenceAsync(GO_TO_STACK_C1);
                                }
                                else if(cazAuto==2){
                                    drive.followTrajectorySequenceAsync(GO_TO_STACK_C2);
                                }
                                else{
                                    drive.followTrajectorySequenceAsync(GO_TO_STACK_C3);
                                }
                            }
                        }
                        else if(nrCycles==3)
                        {
                            if (nrCycles > howManyCycles)
                            {
                                drive.followTrajectorySequenceAsync(PARK_ROBOT);
                            }
                            else
                            {
                                 drive.followTrajectorySequenceAsync(GO_TO_STACK_CYCLE3_C1);
                            }
                        }

                        timerLift.reset();
                        status = STROBOT.RETRACT_LIFT;
                    }
                    break;
                }
                case RETRACT_LIFT:
                {
                    rotateClawController.currentStatus= RotateClawController.RotateStatus.VERTICAL;
                    parbrizController.currentStatus= ParbrizController.ParbrizStatus.CLOSED;
                    outtakeSlidesController.currentStatus=OuttakeSlidesController.ExtensionStatus.COMPACT;
                    turretController.currentStatus= TurretController.TurretStatus.INIT;
                    if (timerLift.seconds()> 0.60 && (liftMotorController.currentStatus != LiftMotorController.LiftStatus.GOING_DOWN && liftMotorController.currentStatus != LiftMotorController.LiftStatus.INIT))
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
                        parbrizController.currentStatus= ParbrizController.ParbrizStatus.CLOSED;
                        status = STROBOT.GO_COLLECT_PIXELS;
                        timerSlides.reset();
                    }

                    break;
                }
                case GO_COLLECT_PIXELS:
                {
                    if(nrCycles==1) timeOpenSlides=1.5;
                    if(nrCycles==3) timeOpenSlides=2;
                    if (timerSlides.seconds()>timeOpenSlides && extenderController.currentStatus != ExtenderController.ExtenderStatus.FAR &&  nrCycles<=howManyCycles)
                    {
                        extenderController.currentStatus = ExtenderController.ExtenderStatus.FAR;
                        if(nrCycles==3)
                        {
                            if (cazAuto == 3)
                            {
                                extenderController.currentStatus= ExtenderController.ExtenderStatus.AUTO_CYCLE_3;
                            }
                            else
                            {
                                extenderController.currentStatus= ExtenderController.ExtenderStatus.AUTO;
                            }
                        }
                        if(DID_FAILSAFE==false){
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE_STACK;
                        }
                        if(nrCycles==3)
                        {
                            collectForbarController.currentStatus= CollectForbarController.CollectStatus.COLLECT_DRIVE_STACK;
                        }
                        if(collectForbarController.currentStatus != CollectForbarController.CollectStatus.COLLECT_DRIVE) {
                            if (nrCycles == 2) {
                                collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_LOW;
                                if(DID_FAILSAFE == true) collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                            }
                        }
                        if(DID_FAILSAFE_TWICE == true) collectForbarController.currentStatus= CollectForbarController.CollectStatus.COLLECT_DRIVE;
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
                    else if(robot.beamBack.getState() == false)
                    {
                        if(nrCycles==1 || nrCycles==3){
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_3;
                        }
                        else if(nrCycles==2)
                        {
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                        }
                        if (timeoutColectare.seconds() > timeout_1pixel) {
                            // Pun timpul pentru extendo
                            transferController.actualTimeForExtendo = TransferController.timerExtendoToInit;
                            extenderController.currentStatus = ExtenderController.ExtenderStatus.INIT;
                            transferController.currentStatus = TransferController.TransferStatus.BLOCHEAZA_TUBULETE;

                            drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD);
                            status = STROBOT.PLACE_STACK_PIXELS_BB;
                            timerLift.reset();
                        }
                        if(DID_FAILSAFE_TWICE == true) collectForbarController.currentStatus= CollectForbarController.CollectStatus.COLLECT_DRIVE;
                    }
                    else if(robot.beamBack.getState() == true && robot.beamFront.getState() == true && extenderController.currentStatus == ExtenderController.ExtenderStatus.FAR)
                    {
                        if (timeoutColectare.seconds() > timeout_nopixel) {
                            collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                            extenderController.currentStatus = ExtenderController.ExtenderStatus.FAILSAFE;
                            status = STROBOT.FAILSAFE_NO_PIXELS;
                        }
                        if(nrCycles==1 && timeoutColectare.seconds() > 0.5) collectForbarController.currentStatus= CollectForbarController.CollectStatus.COLLECT_AUTO_STACK_4;
                    }
                    break;
                }
                case FAILSAFE_NO_PIXELS: {
                    intakeController.currentStatus= IntakeController.IntakeStatus.REVERSE;
                    collectForbarController.currentStatus= CollectForbarController.CollectStatus.PLAY;

                    if (Math.abs(extenderCurrentPosition - ExtenderController.extenderFailsafe) <= 30)
                    {
                        collectForbarController.currentStatus = CollectForbarController.CollectStatus.COLLECT_DRIVE;
                        DID_FAILSAFE = true;
                        status = STROBOT.GO_COLLECT_PIXELS;
                    }
                    break;
                }
                case LEAVE_WITH_2_PIXELS:
                {

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
                        drive.followTrajectorySequenceAsync(GO_PLACE_ON_BACKBOARD_CYCLE3);
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
                        outtakeSlidesController.currentStatus = OuttakeSlidesController.ExtensionStatus.FAR;
                        turretController.currentStatus= TurretController.TurretStatus.RUNTO;
                        rotateClawController.currentStatus = RotateClawController.RotateStatus.RUNTO;
                        timerDrop.reset();
                    }

                    if(liftMotorController.currentPosition<-120)
                    {
                        rotateClawController.targetposition = rotateClawController.horizontalServoPosition - (0.0010 * (0 - ANGLE_SPIKE_LEFT));
                        turretController.targetposition = turretController.initPosition + (0.003 * (0 - ANGLE_SPIKE_LEFT));
                    }

                    if (timerDrop.seconds()> timeToDrop && (liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE1_C1 || liftMotorController.currentStatus==LiftMotorController.LiftStatus.liftAngle || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE2_C1 || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE1_C23 || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE2_C23 || liftMotorController.currentStatus == LiftMotorController.LiftStatus.AUTO_CYCLE3))
                    {
                        parbrizController.currentStatus= ParbrizController.ParbrizStatus.OPEN;
                        sigurantaOuttakeController.currentStatus= SigurantaOuttakeController.SigurantaOuttakeStatus.OPEN;
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
                liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
                rotateClawController.currentStatus= RotateClawController.RotateStatus.VERTICAL;
                turretController.currentStatus = TurretController.TurretStatus.INIT;
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
            outtakeSlidesController.update();
            drive.update();
            telemetry.addData("Pozitie: ", drive.getPoseEstimate());
            telemetry.addData("Cycle: ",nrCycles);
            telemetry.addData("Status",status);
            telemetry.addData("AutoTimer", AutoTimer.seconds());
            telemetry.addData("Extender status", extenderController.currentStatus);
            telemetry.addData("extender position", extenderCurrentPosition);
            telemetry.addData("lift status", liftMotorController.currentStatus);
            telemetry.addData("lift position", liftMotorController.currentPosition);
            telemetry.addData("BeamFront", robot.beamFront.getState());
            telemetry.addData("BeamBack", robot.beamBack.getState());
            telemetry.addData("forbar collect", collectForbarController.currentStatus);
            telemetry.addData("Scoring controller", scoringController.currentStatus);
            telemetry.addData("DID_FAILSAFE", DID_FAILSAFE);
            telemetry.update();
        }
    }

}
//ma omor