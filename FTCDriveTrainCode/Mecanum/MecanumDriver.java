package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.Range;


public class XDriver{
    //declaring variables, for tweaking use sensitivity and top speed
    public DcMotor frontLeftMotor , frontRightMotor, backLeftMotor, backRightMotor;

    double positionPower = 0.7;

    HardwareMap hwMap = null;

    private ElapsedTime runtime = new ElapsedTime();

    
    public void init( HardwareMap ahwMap ) 
    {
        // Save reference to Hardware map
        HardwareMap hwMap = ahwMap;  
        
        frontLeftMotor  = hwMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor  = hwMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");
        
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    } 
        public void drive(double forward, double strafe, double turn){
            
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double leftPowerF = forward - turn - strafe;
            double leftPowerB = forward - turn + strafe;
            double rightPowerF = forward - turn - strafe;
            double rightPowerB = forward - turn + strafe;
            frontLeftMotor.setPower(leftPowerF);
            frontRightMotor.setPower(rightPowerF);
            backLeftMotor.setPower(leftPowerB);
            backRightMotor.setPower(rightPowerB);
        }

        public void driveToPosition(int rb, int rf, int lb, int lf){
            frontLeftMotor.setTargetPosition(lf);
            frontRightMotor.setTargetPosition(rf);
            backLeftMotor.setTargetPosition(lb);
            backRightMotor.setTargetPosition(rb);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            blueMotor.setPower(positionPower);
            redMotor.setPower(positionPower);
            yellowMotor.setPower(positionPower);
            greenMotor.setPower(positionPower);


        }
        
}
