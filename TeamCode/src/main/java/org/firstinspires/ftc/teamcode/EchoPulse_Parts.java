package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.motors.TetrixMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

class EchoPulse_Parts {

    private HardwareMap hardwareMap;

    EchoPulse_Parts(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    DcMotor getMotorSF() {
        return hardwareMap.dcMotor.get("MotorSF");
    }

    DcMotor getMotorDF() {
        return hardwareMap.dcMotor.get("MotorDF");
    }

    DcMotor getMotorDS() {
        return hardwareMap.dcMotor.get("MotorDS");
    }

    DcMotor getMotorSS() {
        return hardwareMap.dcMotor.get("MotorSS");
    }

    DcMotor getMotorCarlig() {
        return hardwareMap.dcMotor.get("MotorCarlig");
    }


    BNO055IMU getGyro() {
        return hardwareMap.get(BNO055IMU.class, "imu");
    }

    WebcamName getWebcam() {
        return hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    DcMotor getDcBaza() {
        return hardwareMap.dcMotor.get("bazaDC");
    }

    DcMotor getDcExtindere() { return hardwareMap.dcMotor.get("extindereDC"); }

    Servo getRotatieCuva() { return hardwareMap.servo.get("rotatieCuva"); }
}
