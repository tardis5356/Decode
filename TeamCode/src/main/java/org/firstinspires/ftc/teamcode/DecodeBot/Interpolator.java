package org.firstinspires.ftc.teamcode.DecodeBot;



/**
 * An interpolation function that returns a value interpolated between an upper and lower bound.
 * This behavior can be linear or nonlinear.
 *
 * @param <T> The type that the {@link Interpolator} will operate on.
 */
@FunctionalInterface
public interface Interpolator<T> {
    /**
     * Perform interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue The value to end at.
     * @param t How far between the two values to interpolate. This should be bounded to [0, 1].
     * @return The interpolated value.
     */
    T interpolate(T startValue, T endValue, double t);

    /**
     * Returns interpolator for Double.
     *
     * @return Interpolator for Double.
     */
    static Interpolator<Double> forDouble() {
        return MathUtil::interpolate;
    }
}