package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

@Config
public class ForbarOuttakeController {
    public enum ForbarStatus{
        GET_COLLECTED_PIXELS, // Cand putem efectua transfer
        PLACE_ON_BACKBOARD, // Cand pui pe backboard
        MOVE_TO_BACKBOARD_DELAY, // Cand vrei sa-l misti cu un delay
        PLACE_ON_BACKBOARD_WITH_DELAY,
        PLACE_ON_BACKBOARD_WITH_ANGLE,
        SET3,
        DOWN,
        PRELOAD;
    }
    public ForbarStatus currentStatus = ForbarStatus.GET_COLLECTED_PIXELS;
    public ForbarStatus previousStatus = null;
    public static double getPixelsPosition = 1;
    public static double placePixelsPosition = 0.67;
    public static double placePixelsAnglePosition = 0.56f;
    public static double  downPosition = 0.25;
    public static double  placePreload = 0.74;
    private ElapsedTime waitForTheLift = new ElapsedTime();
    private double timerPentruForbarOuttake = 0.05; // Modificati asta in functie de cat sa stea forbarul pana se ridica.
    private Servo forbarOuttake = null;
    public ForbarOuttakeController(RobotMap robot)
    {
        this.forbarOuttake = robot.forbarOuttake;
    }
    public void update()
    {
        /// Orice status care are timere in el trebuie sa apara in if.
        if (currentStatus != previousStatus || currentStatus == ForbarStatus.PLACE_ON_BACKBOARD_WITH_DELAY)
        {
            previousStatus = currentStatus;
            switch (currentStatus)
            {
                case GET_COLLECTED_PIXELS:
                {
                    this.forbarOuttake.setPosition(getPixelsPosition);
                    break;
                }
                case PLACE_ON_BACKBOARD:
                {
                    this.forbarOuttake.setPosition(placePixelsPosition);
                    break;
                }
                case MOVE_TO_BACKBOARD_DELAY:
                {
                    waitForTheLift.reset();
                    currentStatus = ForbarStatus.PLACE_ON_BACKBOARD_WITH_DELAY;
                    break;
                }
                case PLACE_ON_BACKBOARD_WITH_DELAY:
                {
                    if (waitForTheLift.seconds()>timerPentruForbarOuttake)
                    {
                        currentStatus = ForbarStatus.PLACE_ON_BACKBOARD;
                    }
                    break;
                }
                case PLACE_ON_BACKBOARD_WITH_ANGLE:
                {
                    if (waitForTheLift.seconds()>timerPentruForbarOuttake) {
                        this.forbarOuttake.setPosition(placePixelsAnglePosition);
                    }
                    break;
                }
                case DOWN:
                {
                    this.forbarOuttake.setPosition(downPosition);
                    break;
                }
                case PRELOAD:
                {
                    this.forbarOuttake.setPosition(placePreload);
                    break;
                }
            }
        }
    }
}
