package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBotGoToStack = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.7, 5.7, 13.22)
                .followTrajectorySequence( drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(PLACE_SPIKE_RIGHT_X,PLACE_SPIKE_RIGHT_Y,Math.toRadians(ANGLE_SPIKE_RIGHT)))
                                .setReversed(true)
                                .splineTo(new Vector2d(GO_TO_STACK_X_C3, GO_TO_STACK_Y_C3),Math.toRadians(180))
                                .lineTo(new Vector2d(COLLECT_STACK_X_C3, COLLECT_STACK_Y_C3))
                                .build()
                );


        RoadRunnerBotEntity myBotToBackdrop = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.7, 5.7, 13.22)
                .followTrajectorySequence( drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(FIRST_PIXEL_X_C3,FIRST_PIXEL_Y_C3,Math.toRadians(-38)))
                                .lineToLinearHeading(new Pose2d(GO_TO_BB_PRELOAD_X,GO_TO_BB_PRELOAD_Y, Math.toRadians(0)))
                                .lineTo(new Vector2d(PLACE_BB_LLH1_X_3, PLACE_BB_LLH1_Y_3))
                                .splineTo(new Vector2d(PLACE_SPIKE_RIGHT_X,PLACE_SPIKE_RIGHT_Y),Math.toRadians(ANGLE_SPIKE_RIGHT))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBotGoToStack)
                .start();

    }
    public static double forwardRight = 6.75;

    public static  double PRELOAD_RIGHT_X = -32.5;
    public static  double PRELOAD_RIGHT_Y = -36;
    public static  double PRELOAD_ANGLE_RIGHT = 39.5;

    public static  double PLACE_SPIKE_RIGHT_X = 49;
    public static  double PLACE_SPIKE_RIGHT_Y = -47;
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
    public static double FIRST_PIXEL_X_C2 =-46.5;
    public static double FIRST_PIXEL_Y_C2 =-19.8;
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
    public static double COLLECT_STACK_X_C3 = -25.5f;
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
    public static final double PLACE_BB_LLH2_Y_CYCLES = -29;
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



}  