package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp


public class lifttestFinal extends LinearOpMode {
    private PIDFController controller;

    public static double p = 0.0035, i = 0.00002, d = 0.0002;
    public static double f = 0.00005;

    public static int target = 0;

    public static double relatieP =0.0004;


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDFController(p, i, d,f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Declaram Motoare
        DcMotor rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        DcMotor leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");

        //Schimbam Directie
        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

            controller.setPIDF(p, i, d,f);
            double liftPos = (rightLift.getCurrentPosition() + leftLift.getCurrentPosition()) / 2;
            double pidf = controller.calculate(liftPos, target);
            double motorRelativeError = Math.abs(leftLift.getCurrentPosition()-rightLift.getCurrentPosition())>10?leftLift.getCurrentPosition()-rightLift.getCurrentPosition():0;

            double rightLiftPower = pidf+relatieP*motorRelativeError;
            double leftLiftPower = pidf-relatieP*motorRelativeError;
            double denom = Math.max(rightLiftPower,Math.max(leftLiftPower,1));

            rightLift.setPower(rightLiftPower/denom);
            leftLift.setPower(leftLiftPower/denom);


            telemetry.addData("target ", target);
            telemetry.addData("pos ", liftPos);
            telemetry.addData("leftPos", leftLift.getCurrentPosition());
            telemetry.addData("rightPos", rightLift.getCurrentPosition());
            telemetry.addData("relativeerror",motorRelativeError);
            telemetry.update();

        }

    }
}
