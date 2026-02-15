package org.firstinspires.ftc.teamcode.main;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PID {
    public DcMotor motor;
    private int motor_port;
    private DcMotorControllerEx motor_controller;
    private float dir;
    public PID(DcMotor pid_motor, float motor_dir, float p, float i, float d) {
        motor = pid_motor;
        dir = motor_dir;
        motor_controller = (DcMotorControllerEx)motor.getController();
        motor_port = motor.getPortNumber();
        PIDCoefficients pid_coef = new PIDCoefficients(p, i, d);
        motor_controller.setPIDCoefficients(motor_port, DcMotor.RunMode.RUN_USING_ENCODER, pid_coef);
    }
    public void setRPM(int rpm) {
        float vel = rpm * 6 * dir;
        motor_controller.setMotorVelocity(motor_port, vel, AngleUnit.DEGREES);
    };
}
