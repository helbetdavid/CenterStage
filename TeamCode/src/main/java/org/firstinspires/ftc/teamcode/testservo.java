package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

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
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp


public class testservo extends LinearOpMode {


    public static double relatieP =0;


    @Override
    public void runOpMode() throws InterruptedException {

        Servo boxSv = hardwareMap.get(Servo.class,"boxSv");

        Servo leftLiftSv = hardwareMap.get(Servo.class,"leftLiftSv");
        Servo rightLiftSv = hardwareMap.get(Servo.class,"rightLiftSv");

        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {

            leftLiftSv.setPosition(relatieP);
            rightLiftSv.setPosition(relatieP);


        }

    }
}
