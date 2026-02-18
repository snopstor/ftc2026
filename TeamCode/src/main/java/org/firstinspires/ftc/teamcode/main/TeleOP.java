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

package org.firstinspires.ftc.teamcode.main;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.follower.Follower;
import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOP", group="Iterative OpMode")
public class TeleOP extends OpMode {
    private double conv = 2.54 * 1.5;
    private double[][] ini_coor = {{157,49,0},{-119.5, 132, -0.9088},{157,-49,0},{-119.5,-132, 0.9088},{-160, 163,0},{-160, -163,0}};
    private int rob_idx=0;
    private int tow_idx=0;
    private Follower follower;
    private DcMotor intake;
    private PID shoot_up;
    private PID shoot_down;

    private DcMotorControllerEx motor_down, motor_up;

    private Servo lock;
    private double lock_open_pos = 0.29;
    private double lock_close_pos = 0.45;
    private int rpm = 285;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        shoot_up = new PID(hardwareMap.get(DcMotor.class, "shoot_up"), 1, 1.1, 0, 1.5);
        shoot_down = new PID(hardwareMap.get(DcMotor.class, "shoot_down"), -1, 1.1, 0, 1.5);

        motor_down  = (DcMotorControllerEx) shoot_down.motor.getController();
        motor_up  = (DcMotorControllerEx) shoot_up.motor.getController();

        lock = hardwareMap.get(Servo.class, "lock"); // close 0.65; open 0.45

        Pose startPose = new Pose(ini_coor[rob_idx+2*tow_idx][0] / conv, ini_coor[rob_idx+2*tow_idx][1] / conv, ini_coor[rob_idx+2*tow_idx][2]);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.getPose();

        double currentX = currentPose.getX(); currentX *= conv;
        double currentY = currentPose.getY(); currentY *= conv;
        double currentHeading = currentPose.getHeading();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * 0.6,
                -gamepad1.left_stick_x * 0.6,
                -gamepad1.right_stick_x * 0.3,
                true
        );

        telemetry.addData("X: ", currentX);
        telemetry.addData("Y: ", currentY);
        telemetry.addData("Rotation: ", currentHeading);

        telemetry.addData("Lower shooter speed: ", motor_down.getMotorVelocity(shoot_down.motor_port)/6);
        telemetry.addData("Upper shooter speed: ", motor_up.getMotorVelocity(shoot_up.motor_port)/6);

        String z="";
        if(tow_idx == 0) z="red";
        else z="blue";

        telemetry.addData("Alliance: ", z," ", tow_idx );
        if(rob_idx == 0) z="small TRNGL";
        else z="BIG TRNGL";
        telemetry.addData("Position: ", z, " ", rob_idx);

        if (gamepad1.leftBumperWasPressed()){
            tow_idx++; tow_idx %= 2;
            Pose startPose = new Pose(ini_coor[rob_idx+2*tow_idx][0] / conv, ini_coor[rob_idx+2*tow_idx][1] / conv, ini_coor[rob_idx+2*tow_idx][2]);
            follower.setPose(startPose);
        }
        if (gamepad1.rightBumperWasPressed()){
            rob_idx++; rob_idx %= 2;
            Pose startPose = new Pose(ini_coor[rob_idx+2*tow_idx][0] / conv, ini_coor[rob_idx+2*tow_idx][1] / conv, ini_coor[rob_idx+2*tow_idx][2]);
            follower.setPose(startPose);
        }
        if (gamepad1.dpadLeftWasPressed()) rpm -= 5;
        if (gamepad1.dpadRightWasPressed()) rpm += 5;

        shoot_down.setRPM(rpm);
        shoot_up.setRPM(rpm);

        if (gamepad1.left_trigger > 0.3) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
        if(gamepad1.right_trigger > 0.3){
            lock.setPosition(lock_open_pos);
        }
        else lock.setPosition(lock_close_pos);
        telemetry.addData("pos", lock.getPosition());
        telemetry.addData("rpm", rpm);
    }
}
