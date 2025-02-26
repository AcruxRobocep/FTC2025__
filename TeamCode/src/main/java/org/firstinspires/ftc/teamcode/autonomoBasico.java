package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class autonomoBasico extends LinearOpMode {
    DcMotor RF;
    DcMotor RB;
    DcMotor LF;
    DcMotor LB;

    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        RF  = hardwareMap.get(DcMotor.class, "M4");
        RB  = hardwareMap.get(DcMotor.class, "M3");
        LF  = hardwareMap.get(DcMotor.class, "M2");
        LB  = hardwareMap.get(DcMotor.class, "M1");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logoFacingDirection,usbFacingDirection);
        imu.initialize(new IMU.Parameters(orientation));

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();
        resetMotors();
        waitForStart();
        forwardDrive(1000);
        //surfDrive(400,true);

        telemetry.addData("M1", RF.getCurrentPosition());
        telemetry.update();








    }

    public void speedDrive(double spd){
        RF.setPower(spd);
        RB.setPower(spd);
        LF.setPower(spd);
        LB.setPower(spd);
    }

    public void forwardDrive(int target){


        speedDrive(0.6);

        RF.setTargetPosition(target);
        RB.setTargetPosition(target);
        LF.setTargetPosition(target);
        LB.setTargetPosition(target);


        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void surfDrive(int target,boolean right){

        if(right){
            RF.setTargetPosition(-target);
            RB.setTargetPosition(target);
            LF.setTargetPosition(target);
            LB.setTargetPosition(-target);

        }else {
            RF.setTargetPosition(target);
            RB.setTargetPosition(-target);
            LF.setTargetPosition(-target);
            LB.setTargetPosition(target);
        }


        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

        public void resetMotors(){
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }



}
