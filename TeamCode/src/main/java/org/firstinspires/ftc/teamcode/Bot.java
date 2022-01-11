package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Error404Test")

public class Bot extends LinearOpMode {
    DcMotor Motor;
    Servo servo;
    BNO055IMU imu;
    @Override
    public  void runOpMode() throws InterruptedException {
        Motor = hardwareMap.get(DcMotor.class, "Motor");
        servo = hardwareMap.get(Servo.class, "servo");
        Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initIMU();
        waitForStart();
        while(!isStopRequested()){
            telemetry.addData("angle", getAngle());
            telemetry.update();
        }
/*
        servo.setPosition(0);
        sleep(10000);
        servo.setPosition(1); //300 градусов, от 0 до 1
*/

    }
// Посмотреть делитель энкодера и умножить на 360
    public void move(int Position) {
        int oldPosition = Motor.getCurrentPosition();
        while (Motor.getCurrentPosition() - oldPosition < Position) {
            Motor.setPower(1);
            telemetry.addData("Encoder", Motor.getCurrentPosition());
            telemetry.update();
        }
        Motor.setPower(0); //-1
    }

    public void initIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            sleep(30);
            telemetry.addData("Wait", "Calibrating");
            telemetry.update();
        }
        telemetry.addData("OK", "Calibrating");
        telemetry.update();
    }

    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
