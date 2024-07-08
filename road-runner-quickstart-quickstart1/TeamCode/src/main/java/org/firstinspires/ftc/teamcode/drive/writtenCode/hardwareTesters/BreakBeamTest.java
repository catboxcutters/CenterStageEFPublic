package org.firstinspires.ftc.teamcode.drive.writtenCode.hardwareTesters;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "BreakBeamTest", group = "Linear Opmode")
//s0Ft ArabEsc
public class BreakBeamTest extends LinearOpMode {
    // SampleMecanumDrive drive;
    DigitalChannel sensor,sensor2;

    public void runOpMode() {
        sensor = hardwareMap.digitalChannel.get("beamfront");
        sensor2 = hardwareMap.digitalChannel.get("beamback");
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
            telemetry.addData("beamfront", ":  " + sensor.getState());
            telemetry.addData("beamback",": " + sensor2.getState());
            telemetry.update();
        }
    }

}