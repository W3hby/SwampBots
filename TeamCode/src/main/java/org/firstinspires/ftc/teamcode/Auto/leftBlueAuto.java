package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "leftBlueAuto")
public class leftBlueAuto extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
//start here:
///////////////////////////////////////forward
        robot.frontLeft.setPower(1);
        robot.frontRight.setPower(1);
        robot.back.setPower(0);
        sleep(2000);
///////////////////////////////////////stop
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.back.setPower(0);
///////////////////////////////////////backward
        robot.frontLeft.setPower(-1);
        robot.frontRight.setPower(-1);
        robot.back.setPower(0);
        sleep(2000);
///////////////////////////////////////stop
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.back.setPower(0);
///////////////////////////////////////left
        robot.frontLeft.setPower(-1);
        robot.frontRight.setPower(1);
        robot.back.setPower(1);
        sleep(500);
///////////////////////////////////////stop
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.back.setPower(0);
///////////////////////////////////////right
        robot.frontLeft.setPower(1);
        robot.frontRight.setPower(-1);
        robot.back.setPower(-1);
        sleep(500);
///////////////////////////////////////stop
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.back.setPower(0);
    }
}