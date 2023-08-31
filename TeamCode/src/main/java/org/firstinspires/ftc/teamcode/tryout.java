package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class tryout extends LinearOpMode {
    private IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("front_left_drive");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("rear_left_drive");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("front_right_drive");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rear_right_drive");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x; // Remember, Y stick value is reversed
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            // Speed control using triggers
            double speedMultiplier = 1.0 - gamepad1.left_trigger + gamepad1.right_trigger;
            // Cap speedMultiplier between 0.1 and 1.0
            speedMultiplier = Math.min(Math.max(speedMultiplier, 0.5), 1.0);

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            //rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator * speedMultiplier;
            double backLeftPower = (rotY - rotX + rx) / denominator * speedMultiplier;
            double frontRightPower = (rotY - rotX - rx) / denominator * speedMultiplier;
            double backRightPower = (rotY + rotX - rx) / denominator * speedMultiplier;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Debugging outputs
            //telemetry.addData("Bot Heading (radians)", botHeading);
            //telemetry.addData("Speed Multiplier", speedMultiplier);
            //telemetry.update();
        }
    }
}
