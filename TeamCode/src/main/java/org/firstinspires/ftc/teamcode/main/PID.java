package org.firstinspires.ftc.teamcode.main;

import static java.lang.Math.abs;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PID {
    public DcMotorEx motor;
    private final int motor_port;
    private final DcMotorControllerEx motor_controller;
    private final float dir;
    private double vel;
    public PID(DcMotor pid_motor, float motor_dir, double p, double i, double d) {
        motor = (DcMotorEx)pid_motor;
        dir = motor_dir;
//        motor_controller = (DcMotorControllerEx)motor.getController();
//        motor_port = motor.getPortNumber();
        PIDCoefficients pid_coeff = new PIDCoefficients(p, i, d);
//        motor_controller.setPIDCoefficients(motor_port, DcMotor.RunMode.RUN_USING_ENCODER, pid_coeff);
//        motor.setPIDCoefficients(pid_coeff);
    }
    public void setRPM(int rpm) {
        vel = rpm * 6 * dir / abs(dir);
        motor_controller.setMotorVelocity(motor_port, vel, AngleUnit.DEGREES);
    };
}
