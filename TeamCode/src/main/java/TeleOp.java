import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws  InterruptedException {
        Robot R = new Robot(hardwareMap, telemetry, this);
        R.init();
        waitForStart();
        while (!isStopRequested()) {
            R.drive(gamepad1.left_stick_y, gamepad1.right_stick_y);
        }
    }
}
