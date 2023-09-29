package org.firstinspires.ftc.teamcode.Hardware;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




public class Robot {
    //Hardware
    public DcMotor frontRight;
    public DcMotor frontLeft;
    public DcMotor back;
    public DcMotor rightWorm;
    public DcMotor leftWorm;
    public DcMotor flywheel;
    public DcMotor wrist;
    public DcMotor encoderLeft;
    public DcMotor encoderRight;
    public DcMotor encoderMid;
    public Servo leftServo;
    public Servo rightServo;
    public Servo droneServo;
    public Servo wrist2;

    public Servo leftHand;
    public Servo rightHand;

    public ElapsedTime runtime = new ElapsedTime();
    HardwareMap hwMap = null;
    public Robot() {

    }
    public void init(HardwareMap h){
        hwMap = h;

        //Initialize hardware
        frontRight = hwMap.dcMotor.get("frontRight");
        frontLeft = hwMap.dcMotor.get("frontLeft");
        back = hwMap.dcMotor.get("back");
        rightWorm = hwMap.dcMotor.get("rightWorm");
        leftWorm = hwMap.dcMotor.get("leftWorm");
        flywheel = hwMap.dcMotor.get("flywheel");
        wrist = hwMap.dcMotor.get("wrist");
        leftServo = hwMap.servo.get("leftServo");
        rightServo = hwMap.servo.get("rightServo");
        droneServo = hwMap.servo.get("droneServo");
        wrist2 = hwMap.servo.get("wrist2");
        leftHand = hwMap.servo.get("leftHand");
        rightHand = hwMap.servo.get("rightHand");
        encoderLeft = frontLeft;
        encoderRight = frontRight;
        encoderMid = back;

        //Set motor direction
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        back.setDirection(DcMotor.Direction.FORWARD);
        rightWorm.setDirection(DcMotor.Direction.REVERSE);
        leftWorm.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        wrist.setDirection(DcMotor.Direction.FORWARD);


        //set power to 0
        frontRight.setPower(0);
        frontLeft.setPower(0);
        back.setPower(0);
        rightWorm.setPower(0);
        rightWorm.setPower(0);
        flywheel.setPower(0);
        wrist.setPower(0);


        //Set zero power behavior on motors
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWorm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftWorm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(STOP_AND_RESET_ENCODER);
        frontLeft.setMode(RUN_WITHOUT_ENCODER);

        frontRight.setMode(STOP_AND_RESET_ENCODER);
        frontRight.setMode(RUN_WITHOUT_ENCODER);

        back.setMode(STOP_AND_RESET_ENCODER);
        back.setMode(RUN_WITHOUT_ENCODER);

        rightWorm.setMode(STOP_AND_RESET_ENCODER);
        rightWorm.setMode(RUN_USING_ENCODER);

        leftWorm.setMode(STOP_AND_RESET_ENCODER);
        leftWorm.setMode(RUN_USING_ENCODER);

        flywheel.setMode(STOP_AND_RESET_ENCODER);
        flywheel.setMode(RUN_USING_ENCODER);

        wrist.setMode(STOP_AND_RESET_ENCODER);
        wrist.setMode(RUN_USING_ENCODER);

        leftServo.setPosition(.75);
        rightServo.setPosition(0.25);
        droneServo.setPosition(1);
        leftHand.setPosition(0);
        rightHand.setPosition(0);
        wrist2.setPosition(0.5);
    }

    public void grabLeft (){
        leftServo.setPosition(.75);
    }
    public void grabRight (){
        rightServo.setPosition(.25);
    }
    public void releaseLeft (){
        leftServo.setPosition(1);
    }
    public void releaseRight (){
        rightServo.setPosition(0);
    }
    public class XyhVector {
        public double x;
        public double y;
        public double h;

        public XyhVector(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }
    }
    private static final int DESIRED_FLYWHEEL_RPM = 5000;
    private static final double PRP = 28;
    private static final double RPM_TOLERANCE = 100;
    private int previousTicks;
    private long previousTime;
    public boolean isLaunchTriggered = false;
    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentMidPosition = 0;
    private int oldRightPosition = 0;
    private int oldLeftPosition=0;
    private int oldMidPosition=0;
    final static double L = 170; //distance between encoder 1 and 2 in mm
    final static double B = 90; //distance between the midpoint of encoder 1 and encoder 2 and encoder 3
    final static double R = 24; //wheel radius in mm
    final static double N = 2000; //encoder tics per revolution, goBuilda encoder
    final static double mm_per_tick = 2.0 * Math.PI * R/N;

    public XyhVector START_POS = new XyhVector(0, 0, Math.toRadians(0));
    public XyhVector pos = new XyhVector(START_POS.x, START_POS.y, START_POS.h);


    public void odometry() {

        oldRightPosition = currentRightPosition;
        oldLeftPosition = currentLeftPosition;
        oldMidPosition = currentMidPosition;

        currentRightPosition = -encoderRight.getCurrentPosition();
        currentLeftPosition = -encoderLeft.getCurrentPosition();
        currentMidPosition = encoderMid.getCurrentPosition();

        int dn1 = currentLeftPosition - oldLeftPosition;
        int dn2 = currentRightPosition - oldRightPosition;
        int dn3 = currentMidPosition - oldMidPosition;

        //has the robot turned a tiny bit?
        double dtheta = mm_per_tick * (dn2-dn1) / L;
        double dx = mm_per_tick * (dn1+dn2) / 2.0;
        double dy = mm_per_tick * (dn3 - (dn2-dn1) * B / L );

        //adds small movements to field coordinates;
        double theta = pos.h + (dtheta / 2.0);
        pos.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;

        pos.h = normDiff(pos.h);

    }

    public void launchDrone() {
        droneServo.setPosition(1);
        flywheel.setPower(1.0);
        while (Math.abs(getFlywheelRPM()) < DESIRED_FLYWHEEL_RPM - RPM_TOLERANCE) {
            sleep(50);
        }
        long startTime = System.currentTimeMillis();
        long durationMillis = 50;
        while(System.currentTimeMillis() - startTime< durationMillis) {
            sleep(50);
        }
        droneServo.setPosition(.8);
        sleep(1000);
        flywheel.setPower(0);
        droneServo.setPosition(1);
    }
    private double getFlywheelRPM() {
        int currentTicks = flywheel.getCurrentPosition();
        long currentTime = System.currentTimeMillis();
        double ticksPerMillis = (double) (currentTicks - previousTicks) / (currentTime - previousTime);

        double RPM = (ticksPerMillis * 60000) / PRP;
        previousTicks = currentTicks;
        previousTime = currentTime;
        return RPM;
    }
    private double normDiff(double angle) {
        while (angle < 0 ){
            angle += 2 * Math.PI;
        }
        return angle % (2 * Math.PI);
    }

    public void sleep(long milliseconds){
        try{
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }



}