package org.firstinspires.ftc.teamcode.Algs;

public class PIDF {

        private double kP, kI, kD, kF;

        private double integralSum = 0;
        private double lastError = 0;
        private long lastTime = 0;

        private double integralLimit = 1.0; // prevents windup

        public PIDF(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            lastTime = System.nanoTime();
        }

        public void setCoefficients(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        }

        public void setIntegralLimit(double limit) {
            this.integralLimit = limit;
        }

        public void reset() {
            integralSum = 0;
            lastError = 0;
            lastTime = System.nanoTime();
        }

        public double calculate(double target, double current) {
            double error = target - current;

            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9; // seconds
            lastTime = now;

            // Integral
            integralSum += error * deltaTime;
            integralSum = clamp(integralSum, -integralLimit, integralLimit);

            // Derivative
            double derivative = (error - lastError) / deltaTime;
            lastError = error;

            // PIDF output
            return (kP * error)
                    + (kI * integralSum)
                    + (kD * derivative)
                    + (kF * target);
        }

        private double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }


