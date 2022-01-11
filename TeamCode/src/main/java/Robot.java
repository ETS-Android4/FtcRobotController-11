import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    BNO055IMU imu;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    LinearOpMode L;
    DcMotor Lm, Rm;

    Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode L){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.L = L;
    }

    public void init(){
        Lm = hardwareMap.get(DcMotor.class, "L");
        Rm = hardwareMap.get(DcMotor.class, "R");
        Lm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            L.sleep(30);
            telemetry.addData("Wait", "Calibrating");
            telemetry.update();
        }
        telemetry.addData("OK", "Calibrating");
        telemetry.update();
    }

    public final void delay(long milliseconds) {
        try{
            Thread.sleep(milliseconds);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    public void drive(double speedL, double speedR){
        Lm.setPower(speedL);
        Rm.setPower(speedR);
    }
}
