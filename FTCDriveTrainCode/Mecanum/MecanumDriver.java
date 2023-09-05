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
    //declaring variables, for tweaking use sensitivity and top speed
    public  DcMotor frontLeftMotor , frontRightMotor, backLeftMotor, backRightMotor;
    public  BNO055IMU imu;
            double heading;
            float IMURESET = 0;
            float desiredAngle = 0;
            

            HardwareMap hwMap = null;

    private ElapsedTime runtime = new ElapsedTime();
    
    public void init( HardwareMap ahwMap ) 
    {
        // Save reference to Hardware map
        HardwareMap hwMap = ahwMap;  
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        
        // Defining and Initializing IMU... Initializing it with the above Parameters...
        imu = hwMap.get( BNO055IMU.class, "IMU" );
        imu.initialize( parameters );
        
        
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
    
    public float getHeading(){   
        double heading = (imu.getAngularOrientation().firstAngle - IMURESET);

        return (float)heading;
    }
        
        
    public void drive(double forward, double strafe, double turn, double autoCorrectVar){
        
        heading = (imu.getAngularOrientation().firstAngle - IMURESET);
        
        double leftPowerF = Range.clip((Math.cos(heading) + Math.sin(heading)),-1,1) * (forward) - Range.clip((Math.cos(heading) - Math.sin(heading)),-1,1) * strafe - (turn + autoCorrectVar);
        double leftPowerB = Range.clip((Math.cos(heading) - Math.sin(heading)),-1,1) * (forward) + Range.clip((Math.cos(heading) + Math.sin(heading)),-1,1) * strafe - (turn + autoCorrectVar);
        double rightPowerF = Range.clip((Math.cos(heading) - Math.sin(heading)),-1,1) * (forward) + Range.clip((Math.cos(heading) + Math.sin(heading)),-1,1) * strafe + (turn + autoCorrectVar);
        double rightPowerB = Range.clip((Math.cos(heading) + Math.sin(heading)),-1,1) * (forward) - Range.clip((Math.cos(heading) - Math.sin(heading)),-1,1) * strafe + (turn + autoCorrectVar);
        frontLeftMotor.setPower(leftPowerF);
        frontRightMotor.setPower(rightPowerF);
        backLeftMotor.setPower(leftPowerB);
        backRightMotor.setPower(rightPowerB);
    }// drive

    
    public double autoCorrect(double turn){
        float difference;
        float heading = (imu.getAngularOrientation().firstAngle - IMURESET);

        if(turn != 0){
            desiredAngle = heading;
        }

        difference = desiredAngle - heading;
        return (-1 / Math.PI) * difference;
        

    }// autocorrect

    

    
        
}
        
        
