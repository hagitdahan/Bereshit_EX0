import java.util.*;

public class TrajectoryInterpolator {
    private TreeMap<Double, Double> data = new TreeMap<>();

    public void addPoint(double altitude, double value) {
        data.put(altitude, value);
    }

    public double interpolate(double alt) {
        if (data.containsKey(alt)) return data.get(alt);

        Map.Entry<Double, Double> lower = data.floorEntry(alt);
        Map.Entry<Double, Double> upper = data.ceilingEntry(alt);

        if (lower == null && upper == null) return 0;
        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();

        double x0 = lower.getKey(), y0 = lower.getValue();
        double x1 = upper.getKey(), y1 = upper.getValue();

        double t = (alt - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }
}
