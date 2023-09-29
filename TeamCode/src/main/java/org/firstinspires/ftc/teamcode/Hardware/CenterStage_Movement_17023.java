package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "CenterStage_Movement_17023", group = "Test")
public class CenterStage_Movement_17023 extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);
        telemetry.addData("status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            double x;
            double y;
            double turn;
            double wristPower;
            double armPower;
            double drivePower;
            double chassisSlowModeMultiplier;
            double frontRightPower;
            double frontLeftPower;
            double backPower;

            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            wristPower = gamepad2.right_stick_y;
            armPower = gamepad2.left_stick_y;
            drivePower = (gamepad1.right_trigger > 0) ? gamepad1.right_trigger : 0.3;
            chassisSlowModeMultiplier = (gamepad1.right_trigger > 0) ? 1 : 0.3;
//Triangle Drive Train
            frontLeftPower = (turn + y - x) * chassisSlowModeMultiplier * drivePower;
            frontRightPower = ( y - turn + x) * chassisSlowModeMultiplier * drivePower;
            backPower = ( turn + (x * .2)) * chassisSlowModeMultiplier * drivePower; //this is the omni wheel

            robot.frontLeft.setPower(frontLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.back.setPower(backPower);

            // arm controls3

            robot.leftWorm.setPower(armPower);
            robot.rightWorm.setPower(armPower);



            robot.wrist.setPower(wristPower * .1);

            if (gamepad1.left_bumper){
                robot.releaseLeft();
                robot.releaseRight();
            }
            if (gamepad1.right_bumper){
                robot.grabRight();
                robot.grabLeft();
            }
            if(gamepad2.x){
                robot.wrist2.setPosition(.83);
            }
            if(gamepad2.b){
                robot.wrist2.setPosition(.5);
            }
            if(gamepad2.dpad_up){
                robot.rightHand.setPosition(0);
            }
            if(gamepad2.dpad_down){
                robot.rightHand.setPosition(.15);
            }
            if(gamepad2.dpad_left){
                robot.leftHand.setPosition(0);
            }
            if(gamepad2.dpad_right){
                robot.leftHand.setPosition(.15);
            }
            int armEncoderValue = robot.leftWorm.getCurrentPosition();
            telemetry.addData("Arm Encoder", armEncoderValue);

            //Hand Controls


            if (gamepad2.y && !robot.isLaunchTriggered) {
                robot.isLaunchTriggered = true;
                robot.launchDrone();
            }else if (!gamepad2.y){
                robot.isLaunchTriggered = false;
            }

            //updates
            robot.odometry();
            telemetry.addData("X Position", robot.pos.x);
            telemetry.addData("Y Position", robot.pos.y);
            telemetry.addData("Heading", Math.toDegrees(robot.pos.h));
            //telemetry.addData("Raw Encoder Left", robot.encoderLeft.getCurrentPosition());
            //telemetry.addData("Raw Encoder Right", robot.encoderRight.getCurrentPosition());
            //telemetry.addData("Raw Encoder Mid", robot.encoderMid.getCurrentPosition());


            sleep(10);
            idle();

            telemetry.update();


        }
    }


}

