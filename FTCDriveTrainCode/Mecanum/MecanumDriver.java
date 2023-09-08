package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class MecanumDriverTest{
    // Declare OpMode members as well as field orienting variables.
    public  DcMotor frontLeftMotor , frontRightMotor, backLeftMotor, backRightMotor;
    public  BNO055IMU imu;
            double heading;
            float IMURESET = 0;
            float desiredAngle;
        
            HardwareMap hwMap = null;

    private ElapsedTime runtime = new ElapsedTime();
    
    public void init( HardwareMap ahwMap ){
        // Save reference to Hardware map
        HardwareMap hwMap = ahwMap;  
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        
        // Defining and Initializing IMU... Initializing it with the above Parameters...
        imu = hwMap.get( BNO055IMU.class, "IMU" );
        imu.initialize( parameters );
        
        //set up motors
        frontLeftMotor  = hwMap.get(DcMotor.class, "leftMotorF");
        frontRightMotor  = hwMap.get(DcMotor.class, "rightMotorF");
        backLeftMotor = hwMap.get(DcMotor.class, "leftMotorB");
        backRightMotor = hwMap.get(DcMotor.class, "rightMotorB");
        
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        

    } 

    
    public float getHeading() {
        return (float) imu.getAngularOrientation().firstAngle - IMURESET;
    }//heading 
        
    public void drive(double forward, double strafe, double turn, float turnCorrection) {
        heading = imu.getAngularOrientation().firstAngle - IMURESET;
        // FO1 and FO2 are for field orienting
        double FO1 = Range.clip((Math.cos(heading) + Math.sin(heading)),-1,1);
        double FO2 = Range.clip((Math.cos(heading) - Math.sin(heading)),-1,1);
        
        double leftPowerF = FO1 * (forward) - FO2 * strafe - (turn + turnCorrection);
        double leftPowerB = FO2 * (forward) + FO1 * strafe - (turn + turnCorrection);
        double rightPowerF = FO2 * (forward) + FO1 * strafe + (turn + turnCorrection);
        double rightPowerB = FO1 * (forward) - FO2 * strafe + (turn + turnCorrection);
        
        frontLeftMotor.setPower(leftPowerF);
        frontRightMotor.setPower(rightPowerF);
        backLeftMotor.setPower(leftPowerB);
        backRightMotor.setPower(rightPowerB);
    }// drive

    public float easing(float difference){
        //Using logistic growth for easing
        float easing = (float) ( 2 * (1/(1 + Math.pow(Math.E, -1.2 * difference)) - 0.5));
        return easing;
    }// easing

    public float turnCorrection(float desiredAngleInput) {
        
        heading = imu.getAngularOrientation().firstAngle - IMURESET;
        float difference = desiredAngleInput - getHeading();
        
        //creates shortest path of correction
        if(Math.abs(difference) > Math.PI) {
            return easing(difference);
        }else{
        return -1 * easing(difference);
    }
    }//turnCorrection
        
}
        
        
