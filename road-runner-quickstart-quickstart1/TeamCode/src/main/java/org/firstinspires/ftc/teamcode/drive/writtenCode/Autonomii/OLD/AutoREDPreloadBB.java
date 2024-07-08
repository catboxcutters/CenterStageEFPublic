package org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.OLD;


import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.OLD.AutoREDPreloadBB.STROBOT.END_AUTO;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.OLD.AutoREDPreloadBB.STROBOT.PARK;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.OLD.AutoREDPreloadBB.STROBOT.PLACE_PURPLE_PIXEL;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.OLD.AutoREDPreloadBB.STROBOT.PLACE_SPIKE_BACKDROP;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.OLD.AutoREDPreloadBB.STROBOT.RETRACT_LIFT;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.Autonomii.OLD.AutoREDPreloadBB.STROBOT.WAIT_FOR_PURPLE_PIXEL;
import static org.firstinspires.ftc.teamcode.drive.writtenCode.controllers.TransferController.initPosition;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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

public class AutoREDPreloadBB extends LinearOpMode {

    public static double forwardLeft = 6.75;
    public static  double PRELOAD_LEFT_X = 15;
    public static  double PRELOAD_LEFT_Y = -36.5;
    public static  double PRELOAD_ANGLE_LEFT = -5;
    public static  double PLACE_SPIKE_LEFT_X = 49;
    public static  double PLACE_SPIKE_LEFT_Y = -31;
    public static  double ANGLE_SPIKE_LEFT = 0;
    public static  double PARK_LEFT_X = 48;
    public static  double PARK_LEFT_Y = -17; //-65
    public static  double ANGLE_PARK_LEFT = 0;
    public static  double PRELOAD_MID_X = 25;
    public static  double PRELOAD_MID_Y = -25;
    public static  double PRELOAD_ANGLE_MID = -5;
    public static  double PLACE_SPIKE_MID_X = 49;
    public static  double PLACE_SPIKE_MID_Y = -37.5;
    public static  double ANGLE_SPIKE_MID = 0;
    public static  double PARK_MID_X = 48;
    public static  double PARK_MID_Y = -17;
    public static  double ANGLE_PARK_MID = 0;
    public static  double PRELOAD_RIGHT_X = 32;
    public static  double PRELOAD_RIGHT_Y = -35.5;
    public static  double PRELOAD_ANGLE_RIGHT = -5;
    public static  double PLACE_SPIKE_RIGHT_X = 49;
    public static  double PLACE_SPIKE_RIGHT_Y = -46;
    public static double ANGLE_SPIKE_RIGHT = 0;
    public static double PARK_RIGHT_X = 48;
    public static double PARK_RIGHT_Y = -17;
    public static double ANGLE_PARK_RIGHT = 0;

    enum STROBOT
    {
        START,
        PLACE_PURPLE_PIXEL,
        WAIT_FOR_PURPLE_PIXEL,
        PLACE_SPIKE_BACKDROP,
        PARK,
        RETRACT_LIFT,
        END_AUTO,
    }
    public static double timePlacePixel = 1;
    public static double delayLift = 0.4;
    public static double waitTimeBackDrop = 1;
    ElapsedTime timerPunerePixel = new ElapsedTime();
    ElapsedTime timerLift = new ElapsedTime();

    CaseDetectionPipeline cameraRecognition;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotMap robot = new RobotMap(hardwareMap);

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
        cameraRecognition = new CaseDetectionPipeline(hardwareMap,telemetry,"red");
        cameraRecognition.initCamera();
        cameraRecognition.start(1);

        Pose2d startPose = new Pose2d(10, -62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        STROBOT status = STROBOT.START;
        TrajectoryVelocityConstraint VELLLH = getVelocityConstraint(40, 5, 12.05);
        TrajectorySequence PLACE_PRELOAD_LEFT = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(VELLLH)
                .lineToLinearHeading(new Pose2d(PRELOAD_LEFT_X, PRELOAD_LEFT_Y,Math.toRadians(PRELOAD_ANGLE_LEFT)))
                .back(forwardLeft)
                .build();
        TrajectorySequence PLACE_SPIKE_LEFT = drive.trajectorySequenceBuilder(PLACE_PRELOAD_LEFT.end())
                .setVelConstraint(VELLLH)
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_LEFT_X, PLACE_SPIKE_LEFT_Y,Math.toRadians(ANGLE_SPIKE_LEFT)))
                .build();
        TrajectorySequence PARK_ROBOT_LEFT = drive.trajectorySequenceBuilder(PLACE_SPIKE_LEFT.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(PARK_LEFT_X, PARK_LEFT_Y,Math.toRadians(ANGLE_PARK_LEFT)))
                .strafeLeft(5)
                .forward(10)
                .build();


        TrajectorySequence PLACE_PRELOAD_MID = drive.trajectorySequenceBuilder(startPose)
                //.setVelConstraint(VELLLH)
                .lineToLinearHeading(new Pose2d(PRELOAD_MID_X, PRELOAD_MID_Y,Math.toRadians(PRELOAD_ANGLE_MID)))
                .build();
        TrajectorySequence PLACE_SPIKE_MID = drive.trajectorySequenceBuilder(PLACE_PRELOAD_MID.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_MID_X, PLACE_SPIKE_MID_Y,Math.toRadians(ANGLE_SPIKE_MID)))
                .build();
        TrajectorySequence PARK_ROBOT_MID = drive.trajectorySequenceBuilder(PLACE_SPIKE_MID.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(PARK_MID_X, PARK_MID_Y,Math.toRadians(ANGLE_PARK_MID)))
                .strafeLeft(5)
                .forward(10)
                .build();


        TrajectorySequence PLACE_PRELOAD_RIGHT = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(PRELOAD_RIGHT_X, PRELOAD_RIGHT_Y,Math.toRadians(PRELOAD_ANGLE_RIGHT)))
                .build();
        TrajectorySequence PLACE_SPIKE_RIGHT = drive.trajectorySequenceBuilder(PLACE_PRELOAD_RIGHT.end())
                .lineToLinearHeading(new Pose2d(PLACE_SPIKE_RIGHT_X, PLACE_SPIKE_RIGHT_Y,Math.toRadians(ANGLE_SPIKE_RIGHT)))
                .build();
        TrajectorySequence PARK_ROBOT_RIGHT = drive.trajectorySequenceBuilder(PLACE_SPIKE_RIGHT.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(PARK_RIGHT_X, PARK_RIGHT_Y,Math.toRadians(ANGLE_PARK_RIGHT)))
                .strafeLeft(5)
                .forward(10)
                .build();
        double cazAuto = 1;
        while (!isStarted()&&!isStopRequested())
        {
            int c =  cameraRecognition.getCase();
            telemetry.addData("detected",c);
            cazAuto=c;
            telemetry.addLine("Init Complete");
            telemetry.update();
            sleep(50);
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
                    status = PLACE_PURPLE_PIXEL;
                    break;
                }
                case PLACE_PURPLE_PIXEL:
                {
                    if (!drive.isBusy())
                    {
                        intakeController.currentStatus = IntakeController.IntakeStatus.REVERSE_AUTO;
                        timerPunerePixel.reset();
                        status = WAIT_FOR_PURPLE_PIXEL;
                    }
                    break;
                }
                case WAIT_FOR_PURPLE_PIXEL:
                {
                    if (timerPunerePixel.seconds()>timePlacePixel)
                    {
                        intakeController.currentStatus = IntakeController.IntakeStatus.STOP;
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
                        status = PLACE_SPIKE_BACKDROP;
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
                        status = PARK;
                    }
                    break;
                }
                case PARK:
                {
                    if (timerPunerePixel.seconds() > waitTimeBackDrop)
                    {
                        if (cazAuto == 1)
                        {
                            drive.followTrajectorySequenceAsync(PARK_ROBOT_LEFT);
                        }
                        else if (cazAuto ==2)
                        {
                            drive.followTrajectorySequenceAsync(PARK_ROBOT_MID);
                        }
                        else if (cazAuto == 3)
                        {
                            drive.followTrajectorySequenceAsync(PARK_ROBOT_RIGHT);
                        }
                        timerLift.reset();
                        status = RETRACT_LIFT;
                    }
                    break;
                }
                case RETRACT_LIFT:
                {
                    if (timerLift.seconds()> 0.3)
                    {
                        liftMotorController.currentStatus = LiftMotorController.LiftStatus.INIT;
                        status = END_AUTO;
                    }
                    break;
                }
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
            telemetry.addData("Status",status);
            telemetry.update();
        }
    }

}