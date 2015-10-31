package ros.msgs.geometry_msgs;

/**
 * @author James MacGlashan.
 */
public class Twist {
	public Vector3 linear = new Vector3();
	public Vector3 angular = new Vector3();

	public Twist(){}

	public Twist(Vector3 linear, Vector3 angular) {
		this.linear = linear;
		this.angular = angular;
	}
}
