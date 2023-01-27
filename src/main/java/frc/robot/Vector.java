package frc.robot;

public class Vector {
    public double x;
    public double y;

    public final double length;

    /**
     * Creates a new vector with the specified x and y coordinates.
     *
     * @param x The x coordinate
     * @param y The y coordinate
     *
     * @since 0.1
     */
	public Vector(double x, double y) {
		this.x = x;
		this.y = y;

		this.length = Math.hypot(x, y);
	}

    /**
	 * Multiplies each component of the vector by a scalar value.
	 * @param scalar The scalar to multiply each component by.
	 * @return The vector scaled by the scalar.
	 */
	public Vector scale(double scalar) {
		return multiply(scalar, scalar);
	}

    /**
     * Preforms a component-wise multiplication on this vector with another vector.
     *
     * @param vector The vector to multiply by
     * @return A vector with the result of the multiplication
     * @since 0.1
     */
	public Vector multiply(Vector vector) {
		return multiply(vector.x, vector.y);
	}

    /**
     * Multiplies the components of this vector by two scalar values.
     *
     * @param x A scalar to multiply the x-coordinate by
     * @param y A scalar to multiply the y-coordinate by
     * @return A vector with the result of the multiplication
     * @since 0.1
     */
	public Vector multiply(double x, double y) {
		return new Vector(this.x * x, this.y * y);
	}

    /**
     * Gets the angle of the vector.
     *
     * @return A rotation representing the vector's angle
     * @since 0.2
     */
	public Rotation2 getAngle() {
		return new Rotation2(x, y, true);
	}

    /**
	 * Creates a unit vector from a rotation.
     *
	 * @param rotation The rotation to create the vector from
	 * @return A unit vector with the specified angle.
     * @since 0.2
	 */
	public static Vector fromAngle(Rotation2 rotation) {
		return new Vector(rotation.cos, rotation.sin);
	}

    /**
     * Calculates the cross product of this vector and another vector in 3d space and returns the length.
     *
     * @param other The other vector to calculate the cross product with
     * @return The length of the calculated vector
     * @since 0.2
     */
	public double cross(Vector other) {
		return x * other.y - y * other.x;
	}
}
