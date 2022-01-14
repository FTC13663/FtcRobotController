package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTest extends OpMode {
    DcMotor fl, fr, bl, br;


    @Override
    public void init() {
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        br = hardwareMap.get(DcMotor.class, "rightBack");
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
            fl.setPower(.1);
        } else {
            fl.setPower(0);
        }
    }
}
