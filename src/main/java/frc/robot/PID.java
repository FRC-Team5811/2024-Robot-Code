package frc.robot;

public class PID {
    private final double p;
    private final double i;
    private final double d;
    
    private double distance = 1;
    private double target = 1;
    private double error = distance - target;
    private double f = 0;
    private double accumulation = 0;
    private double lastTime = 0;
    private boolean continuousInput = false;
    private double lowerLim = 0;
    private double upperLim = 1;

    public PID (double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public double calculate (double distance, double target) {
        if (continuousInput) {
            if (distance < lowerLim) {
                    distance = upperLim - (lowerLim - distance) % (upperLim - lowerLim);
                }
            else if (distance > upperLim) {
                    distance = lowerLim + (distance - upperLim) % (upperLim - lowerLim);
            }
            if (target < lowerLim) {
                target = upperLim - (target - distance) % (upperLim - lowerLim);
            } 
            else if (target > upperLim) {
                target = lowerLim + (target - upperLim) % (upperLim - lowerLim);
            }
        }
        double lastError = error;
        error = target - distance;
        if (continuousInput) {
            if (error < lowerLim) {
                    error = upperLim - ((lowerLim - error) % (upperLim - lowerLim));
                } 
            else if (error > upperLim) {
                    error = lowerLim + ((error - upperLim) % (upperLim - lowerLim));
            }
        }
        double derivative = (error - lastError)/0.02;
        accumulation += error*0.02;
        return p*error + d*derivative + i*accumulation;
    }

    public void enableContinuousInput (double lowerInput, double upperInput) {
        continuousInput = true;
        lowerLim = lowerInput;
        upperLim = upperInput;
    }
}