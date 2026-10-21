package militarylogistics;

public class KalmanFilter {
	private double estimate;
    private double estimateError;
    private double measurementError;
    private double kalmanGain;

    public KalmanFilter(double initialEstimate, double estimateError, double measurementError) {
        this.estimate = initialEstimate;
        this.estimateError = estimateError;
        this.measurementError = measurementError;
    }

    public double update(double measurement) {
        // Calculate Kalman Gain
        kalmanGain = estimateError / (estimateError + measurementError);
        // Update estimate
        estimate = estimate + kalmanGain * (measurement - estimate);
        // Update error
        estimateError = (1 - kalmanGain) * estimateError;
        return estimate;
    }
}

