package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;
import java.util.Stack;

public class VelocityCompensatedSquIDController {
    private double maxDecel;
    private double p;
    private double frictionGain;
    private double lastPv;
    private boolean firstLoop = true;
    private LoopTimeAverager loopTimeAverager;

    {
        loopTimeAverager = new LoopTimeAverager(1);
        lastPv = 0;
    }

    public VelocityCompensatedSquIDController(double maxDecel, double p, double frictionGain) {
        this.maxDecel = maxDecel;
        this.p = p;
        this.frictionGain = frictionGain;
    }

    public void setP(double p) {
        this.p = p;
    }

    public void setMaxDecel(double maxDecel) {
        this.maxDecel = maxDecel;
    }

    public void setFrictionGain(double frictionGain) {
        this.frictionGain = frictionGain;
    }

    public double calculate(double setpoint, double pv) {
        double error = setpoint - pv;

        // v_f^2 = v_i^2 + 2ad for v_f = 0, d = error, and a = maxDecel
        double targetVelocity = Math.signum(error) * Math.sqrt(Math.abs(2 * maxDecel * error));

        /**
         * Expected loop times of the next loop.
         */
        double deltaSeconds = loopTimeAverager.poll();
        double lastVel = (pv - lastPv) / loopTimeAverager.getLast();
        lastPv = pv;

        // compensate for distance travelled, assume constant accel
        // d = d_0 + v * deltaTime
        // subtract targetVelocity because the sign is the wrong way
        error += deltaSeconds * (lastVel - targetVelocity) / 2;

        // recompute target velocity for the next timestamp instead of this one
        targetVelocity = Math.signum(error) * Math.sqrt(Math.abs(2 * maxDecel * error));

        // v_i = v_0 + a * deltaT
        double targetAccel = (targetVelocity - lastVel) / deltaSeconds;

        // a = F/m = (F_motor - F_fric)/m
        // F_motor = ma + F_fric
        return targetAccel * p + frictionGain;
    }

    public class LoopTimeAverager {

        private Stack<Double> times;
        private ElapsedTime loopTimer;
        private double intervalToAverageSeconds;

        {
            times = new Stack<>();
            loopTimer = new ElapsedTime();
        }

        public LoopTimeAverager(double intervalToAverageSeconds) {
            this.intervalToAverageSeconds = intervalToAverageSeconds;
        }

        public void resetTimer() {
            times = new Stack<>();
            loopTimer.reset();
        }

        /**
         * @return Running average of loop times in seconds.
         */
        public double poll() {
            return 0.02;
            /*times.push(loopTimer.seconds());

            double sum = 0;
            for (double t : times) {
                sum += t;
            }

            while (sum > intervalToAverageSeconds) {
                sum -= times.pop();
            }

            loopTimer.reset();

            return sum / times.size();*/
        }

        /**
         * @return The last loop time.
         */
        public double getLast() {
            if (times.isEmpty()) {
                return 0.01;
            }
            return 0.02;
            //return times.elementAt(times.size() - 1);
        }

    }
}
