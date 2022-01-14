package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@TeleOp(name = "Motor Test", group = "test")
public class MotorTest extends OpMode {
    DcMotor fl, fr, bl, br, lift, spinning;
    CRServo intake;

    private BNO055IMU imu;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        br = hardwareMap.get(DcMotor.class, "rightBack");
        spinning = hardwareMap.get(DcMotor.class, "spinning");
        lift = hardwareMap.get(DcMotor.class, "lift");
        intake = hardwareMap.get(CRServo.class, "intake");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinning.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinning.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        CalibrateGyro cbgyro = new CalibrateGyro(true);
        imu = cbgyro.initGyro(hardwareMap);
    }

    @Override
    public void start() {
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinning.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            fl.setPower(.1);
        } else {
            fl.setPower(0);
        }
        if (gamepad1.x) {
            bl.setPower(.1);
        } else {
            bl.setPower(0);
        }
        if (gamepad1.b) {
            fr.setPower(.1);
        } else {
            fr.setPower(0);
        }
        if (gamepad1.y) {
            br.setPower(.1);
        } else {
            br.setPower(0);
        }

        lift.setPower(gamepad1.right_stick_y);
        spinning.setPower(gamepad1.right_trigger);
        intake.setPower(gamepad1.left_stick_y);


        Orientation cur = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("HEADING Z: ", cur.firstAngle);
        telemetry.addData("HEADING Y: ", cur.secondAngle);
        telemetry.addData("HEADING X: ", cur.thirdAngle);
        telemetry.addData("BL Enc: ", bl.getCurrentPosition());
        telemetry.addData("FL Enc: ", fl.getCurrentPosition());
        telemetry.addData("BR Enc: ", br.getCurrentPosition());
        telemetry.addData("FR Enc: ", fr.getCurrentPosition());
        telemetry.addData("Lift Enc: ", lift.getCurrentPosition());
        telemetry.addData("Duck Enc: ", spinning.getCurrentPosition());
        telemetry.update();
    }
}
