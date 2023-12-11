package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
@Config
public class opmode extends LinearOpMode {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    private PIDFController controller;
    public static double p = 0.0035, i = 0.00002, d = 0.0002;
    public static double f = 0.00005;
    public static int target = 0;
    public static double relatieP =0.0004;

    public enum RobotState{
        START,
        COLLECTING,
        NEUTRAL,
        SCORRING,
        RETRACTING
    }
    RobotState robotState= RobotState.START;

    public static double IntakeLowSvPos = 0.55;
    public static double IntakeMidSvPos = 0.22;

    public static double LiftLowSvPos = 0;
    public static double LiftHighSvPos = 0.9;

    boolean liftToggle = false;



    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDFController(p, i, d,f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // Motors DriveTrain
        DcMotor leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotor leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        DcMotor rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // Motors Lift
        DcMotor rightLift = hardwareMap.get(DcMotorEx.class,"rightLift");
        DcMotor leftLift = hardwareMap.get(DcMotorEx.class,"leftLift");

        // Motors Intake
        DcMotor intakeBack = hardwareMap.get(DcMotorEx.class,"intakeBack");

        // Servos Intake
        Servo rightIntakeSv = hardwareMap.get(Servo.class,"rightIntakeSv");
        Servo leftIntakeSv = hardwareMap.get(Servo.class,"leftIntakeSv");

        // Servos Lift
        Servo leftLiftSv = hardwareMap.get(Servo.class,"leftLiftSv");
        Servo rightLiftSv = hardwareMap.get(Servo.class,"rightLiftSv");
        Servo boxSv = hardwareMap.get(Servo.class,"boxSv");

        // Reverse Motors
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set Lift servo's to 0.55 IDLE.
        leftLiftSv.setPosition(0.90);
        rightLiftSv.setPosition(0.90);

        // Wait MARESTI DACA LA INT SE LOVESC SAU CEVAAAAAA!!!!!!!!!
        sleep(750);

        // Set Intake servo's to IntakeMidSvPos.
        leftIntakeSv.setPosition(0);
        rightIntakeSv.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            controller.setPIDF(p, i, d,f);
            double liftPos = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition()) / 2;
            double pidf = controller.calculate(liftPos, target);
            double motorRelativeError = Math.abs(leftLift.getCurrentPosition()-rightLift.getCurrentPosition())>10?leftLift.getCurrentPosition()-rightLift.getCurrentPosition():0;
            double rightLiftPower = pidf+relatieP*motorRelativeError;
            double leftLiftPower = pidf-relatieP*motorRelativeError;
            double fixer = Math.max(rightLiftPower,Math.max(leftLiftPower,1));

            rightLift.setPower(rightLiftPower/fixer);
            leftLift.setPower(leftLiftPower/fixer);

            switch (robotState){
                case START:
                    leftIntakeSv.setPosition(IntakeMidSvPos);
                    rightIntakeSv.setPosition(IntakeMidSvPos);
                    leftLiftSv.setPosition(0.50);
                    rightLiftSv.setPosition(0.50);
                    if(gamepad2.x){
                        robotState = RobotState.COLLECTING;
                    }
                    break;
                case COLLECTING:
                    boxSv.setPosition(0); // VEDETI AICI CE POZITIE ARE cAND ESTE INCHIS DE PREFERAT 0
                    leftIntakeSv.setPosition(IntakeLowSvPos);
                    rightIntakeSv.setPosition(IntakeLowSvPos);
                    wait(750); // Wait MARESTI  SE LOVESC SAU CEVAAAAAA!!!!!!!!!
                    leftLiftSv.setPosition(LiftLowSvPos);
                    rightLiftSv.setPosition(LiftLowSvPos);
                    wait(200);
                    intakeBack.setPower(1);
                    if(gamepad2.y){
                        robotState = RobotState.NEUTRAL;
                    }
                    break;
                case NEUTRAL:
                    intakeBack.setPower(0);
                    leftLiftSv.setPosition(LiftHighSvPos);
                    rightLiftSv.setPosition(LiftHighSvPos);
                    wait(750);// Wait MARESTI  SE LOVESC SAU CEVAAAAAA!!!!!!!!!
                    leftIntakeSv.setPosition(IntakeMidSvPos);
                    rightIntakeSv.setPosition(IntakeMidSvPos);
                    if(gamepad2.b){
                        robotState = RobotState.SCORRING;
                    }
                    break;
                case SCORRING:
                    target=1200; //MARITI DACA VRETI MAI SUS!!!
                    if(gamepad1.a){
                        boxSv.setPosition(1); //VEDETI AICI CE POZITIE ARE CAND E DESCHIS!!!!!!!
                        wait(1000); //ASTEAPTA O SECUNDA DUPA CE SE DESCHIDE CUTIA
                        target = 0;
                        robotState = RobotState.RETRACTING;
                    }
                    break;
                case RETRACTING:
                    if(liftPos<=15){
                        robotState = RobotState.START;
                    }
                    break;
                default:
                    robotState = RobotState.START;
            }

            //FUNCTIE IN CAZ DE NU MERGE CEVA S AU S A BLOCAT CEVA SA INCEPETI PROCESUL DE LA CAPAT!!!!
            if(gamepad1.dpad_down && robotState!= RobotState.START){
                robotState = RobotState.START;
            }

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);


            telemetry.addData("Stadiu Robot",robotState);
            telemetry.update();

        }
    }
}