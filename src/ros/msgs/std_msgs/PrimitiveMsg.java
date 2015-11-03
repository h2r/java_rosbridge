package ros.msgs.std_msgs;

/**
 * A generic specified Java Bean for capturing many of the primitive data-type messages used by ROS in the std_msgs
 * package. The class has a single public data member called "data" that belongs to the specified primitive type.
 * @author James MacGlashan.
 */
public class PrimitiveMsg <T> {
	public T data;
	public PrimitiveMsg(){}
	public PrimitiveMsg(T data){this.data = data;}
}
