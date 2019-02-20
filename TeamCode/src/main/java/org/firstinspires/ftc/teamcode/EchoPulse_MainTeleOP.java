/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.EchoPulse_Constants.*;
import static org.firstinspires.ftc.teamcode.EchoPulse_Constants.Behaviour.*;
import static org.firstinspires.ftc.teamcode.EchoPulse_Constants.Direction.*;

@TeleOp(name = "MainTeleOP", group = "FTC")
//@Disabled
public class EchoPulse_MainTeleOP extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorSF, motorDF, motorDS, motorSS, motorCarlig, armBase, armExt;
    private Servo rotatieCuva;

    private double speedDevider = 1;
    private boolean toggleSpeed = false;//opreste inertia
    private boolean toggleBrake = false;//opreste instant
    private boolean useLimits = true;
    private int limitEnd;
    private int limitStart;
    private double rotPower = 0.75;
    private int s = 0;
    private boolean DEBUG = false;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("IT", "FUCKING WORKED");
        telemetry.update();

        EchoPulse_Parts parts = new EchoPulse_Parts(hardwareMap);
        motorSF = parts.getMotorSF();
        motorDF = parts.getMotorDF();
        motorDS = parts.getMotorDS();
        motorSS = parts.getMotorSS();
        motorCarlig = parts.getMotorCarlig();
        armBase = parts.getDcBaza();
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExt = parts.getDcExtindere();
        armExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotatieCuva = parts.getRotatieCuva();
        motorSF.setDirection(DcMotor.Direction.FORWARD);
        motorDF.setDirection(DcMotor.Direction.FORWARD);
        motorDS.setDirection(DcMotor.Direction.FORWARD);
        motorSS.setDirection(DcMotor.Direction.FORWARD);
        motorCarlig.setDirection(DcMotor.Direction.FORWARD);
        motorCarlig.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCarlig.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBase.setDirection(DcMotorSimple.Direction.FORWARD);
        armBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**
         * @return Controller Commands
         *
         * Gamepad#1
         * Left Stick - Moves forward or backward
         * Dpad Left|Right - Moves horizontally
         * Left|Right Bumper - Steering on spot
         * B - Change motors behaviour to 'float' or 'break'
         * A - Enables/Disables Slow mode
         *
         * Gamepad#2
         * Dpad down- Get down the hook
         * Dpad up - Lift the hook
         * Left Stick - Control Arm's Base
         * Right Stick - Extend/Retract Arm
         * Left/Right Trigger - Control Arm's Top
         *
         */


        // Wait for the game to start (driver presses PLAY)
        runtime.reset();
        waitForStart();

        if (opModeIsActive()) {
            limitStart = motorCarlig.getCurrentPosition();
            limitEnd = motorCarlig.getCurrentPosition() - 11000;
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double sasiuPowerY = (-gamepad1.left_stick_y / 1.2) / speedDevider;
                double sasiuPowerX = (gamepad1.left_stick_x / 1.5) / speedDevider;

                speedDevider();

                setLimits();

                double carligPower = 0.8;

                if (gamepad1.left_stick_y == 0) {
                    motorSF.setPower(0);
                    motorDF.setPower(0);
                    motorDS.setPower(0);
                    motorSS.setPower(0);
                }

                if (gamepad1.x && motorSF.getDirection() == DcMotor.Direction.FORWARD)
                    setDirection(DcMotor.Direction.REVERSE);
                if (gamepad1.x && motorSF.getDirection() == DcMotor.Direction.REVERSE)
                    setDirection(DcMotor.Direction.FORWARD);

                if (gamepad2.left_stick_y > 0)
                    armBase.setPower(gamepad2.left_stick_x / 3);
                if (gamepad2.left_stick_y < 0)
                    armBase.setPower(gamepad2.left_stick_x / 3);

                if (gamepad2.right_stick_y > 0)
                    armExt.setPower(gamepad2.right_stick_y / 1.35);
                if (gamepad2.right_stick_y < 0)
                    armExt.setPower(gamepad2.right_stick_y / 1.35);
                if (gamepad2.left_stick_y == 0)
                    armBase.setPower(0);
                if (gamepad2.right_stick_y == 0)
                    armExt.setPower(0);

                if (gamepad2.left_trigger > 0)
                    rotatieCuva.setPosition(rotatieCuva.getPosition() + gamepad2.left_trigger / 100);
                if (gamepad2.right_trigger > 0)
                    rotatieCuva.setPosition(rotatieCuva.getPosition() - gamepad2.right_trigger / 100);

                if (-gamepad1.left_stick_y > 0.2 || -gamepad1.left_stick_y < -0.2) {
                    motorSF.setPower(-sasiuPowerY);
                    motorDF.setPower(sasiuPowerY);
                    motorDS.setPower(sasiuPowerY);
                    motorSS.setPower(-sasiuPowerY);
                }

                implementSteering();

                // Telemetry
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                if (DEBUG) debug(sasiuPowerX);
                if (limitStart != 0 && limitEnd != -11000)
                    telemetry.addData("Limita Actualizata la : ", limitStart + " | " + limitEnd);
                telemetry.addData("Limite: ", useLimits ? "Activate" : "Dezactivate");
                telemetry.addData("MotorCarlig: ", motorCarlig.getCurrentPosition());
                telemetry.addData("Baza DC", armBase.getCurrentPosition());
                telemetry.addData("Extindere DC", armExt.getCurrentPosition());
                telemetry.addData("Rotatie Cuva", rotatieCuva.getPosition());
                telemetry.update();

                useCarlig(carligPower);
            }
        }
    }

    private void setDirection(DcMotor.Direction direction) {
        motorSF.setDirection(direction);
        motorDF.setDirection(direction);
        motorDS.setDirection(direction);
        motorSS.setDirection(direction);
    }

    private void debug(double sasiuPowerX) {
        telemetry.addData("MotorSF: ", motorSF.getCurrentPosition());
        telemetry.addData("MotorDF: ", motorDF.getCurrentPosition());
        telemetry.addData("MotorDS: ", motorDS.getCurrentPosition());
        telemetry.addData("MotorSS: ", motorSS.getCurrentPosition());
        telemetry.addData("sasiuPowerX ", sasiuPowerX);
    }

    private void setLimits() {
        if (gamepad2.y)
            useLimits = false;

        if (gamepad2.x)
            useLimits = true;
    }

    private void implementSteering() {
        if (gamepad1.left_stick_x < -0.2)
            setSteering(HLEFT, false);

        if (gamepad1.left_stick_x > 0.2)
            setSteering(HRIGHT, false);

        if (gamepad1.left_bumper) {
            if (speedDevider == 1)
                setSteering(LEFT, false);
            else
                setSteering(LEFT, true);
        }
        if (gamepad1.right_bumper) {
            if (speedDevider == 1)
                setSteering(RIGHT, false);
            else
                setSteering(RIGHT, true);
        }
    }

    private void setSteering(Direction direction, boolean useDivider) {
        switch (direction) {
            case LEFT:
                if (!useDivider) {
                    motorSF.setPower(rotPower);
                    motorDF.setPower(rotPower);
                    motorDS.setPower(rotPower);
                    motorSS.setPower(rotPower);
                } else {
                    motorSF.setPower(rotPower / 1.25);
                    motorDF.setPower(rotPower / 1.25);
                    motorDS.setPower(rotPower / 1.25);
                    motorSS.setPower(rotPower / 1.25);
                }
                s++;
                break;
            case RIGHT:
                if (!useDivider) {
                    motorSF.setPower(-rotPower);
                    motorDF.setPower(-rotPower);
                    motorDS.setPower(-rotPower);
                    motorSS.setPower(-rotPower);
                } else {
                    motorSF.setPower(-rotPower / 1.25);
                    motorDF.setPower(-rotPower / 1.25);
                    motorDS.setPower(-rotPower / 1.25);
                    motorSS.setPower(-rotPower / 1.25);
                }
                s--;
                break;
            case HLEFT:
                motorSF.setPower(rotPower);
                motorDF.setPower(rotPower);
                motorDS.setPower(-rotPower);
                motorSS.setPower(-rotPower);
                s++;
                break;
            case HRIGHT:
                motorSF.setPower(-rotPower);
                motorDF.setPower(-rotPower);
                motorDS.setPower(rotPower);
                motorSS.setPower(rotPower);
                s--;
                break;
        }
    }

    private void speedDevider() {
        if (gamepad1.b && toggleBrake) {
            setBehaviour(BREAK);
            toggleBrake = false;
        } else if (gamepad1.b && !toggleBrake) {
            setBehaviour(FLOAT);
            toggleBrake = true;
        }

        if (gamepad1.a && toggleSpeed) {
            speedDevider = 2.75;
            rotPower = 0.2;
            toggleSpeed = false;
        } //SlowMode ON
        else if (gamepad1.a && !toggleSpeed) {
            speedDevider = 1;
            rotPower = 0.75;
            toggleSpeed = true;
        } //SlowMode OFF
    }

    private void useCarlig(double carligPower) {
        if (useLimits) {
            if (gamepad2.dpad_down && motorCarlig.getCurrentPosition() < limitStart) {
                motorCarlig.setPower(carligPower);
            } else if (gamepad2.dpad_up && motorCarlig.getCurrentPosition() > limitEnd) {
                motorCarlig.setPower(-carligPower);
            } else {
                motorCarlig.setPower(0);
            }
        } else {
            if (gamepad2.dpad_down)
                motorCarlig.setPower(carligPower);
            else if (gamepad2.dpad_up)
                motorCarlig.setPower(-carligPower);
            else motorCarlig.setPower(0);
        }
    }

    private void setBehaviour(EchoPulse_Constants.Behaviour behaviour) {
        switch (behaviour) {
            case BREAK:
                motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            case FLOAT:
                motorSF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorDF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorDS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
        }
    }
}
