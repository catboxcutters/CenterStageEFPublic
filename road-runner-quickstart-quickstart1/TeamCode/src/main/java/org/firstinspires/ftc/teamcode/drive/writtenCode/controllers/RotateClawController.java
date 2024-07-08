package org.firstinspires.ftc.teamcode.drive.writtenCode.controllers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.writtenCode.RobotMap;

public class RotateClawController {
    public enum RotateStatus{
        VERTICAL,
        HORIZONTAL,
        RUNTO,
    }

    public RotateStatus currentStatus = RotateStatus.VERTICAL;
    public RotateStatus previousStatus = null;

    public static double verticalServoPosition = 0.12;
    public static double horizontalServoPosition = 0.42;//0.395
        private Servo clawRotate = null;
    public double currentposition = 0;
        public double targetposition = verticalServoPosition;
        public double previousPosition;
    public RotateClawController(RobotMap robot)
    {
        clawRotate = robot.clawRotate;
    }
    public void update()
    {
        currentposition=this.clawRotate.getPosition();
        if (currentStatus != previousStatus || previousPosition != targetposition)
        {
            previousStatus = currentStatus;
            previousPosition = targetposition;
            switch (currentStatus)
            {
                case VERTICAL:
                {
                    this.clawRotate.setPosition(verticalServoPosition);
                    break;
                }
                case HORIZONTAL:
                {
                    this.clawRotate.setPosition(horizontalServoPosition);
                    break;
                }
                case RUNTO:
                {
                    this.clawRotate.setPosition(targetposition);
                    break;
                }
            }
        }
    }
}
