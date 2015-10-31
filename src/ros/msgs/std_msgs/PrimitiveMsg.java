package ros.msgs.std_msgs;

/**
 * @author James MacGlashan.
 */
public class PrimitiveMsg <T> {
	public T data;
	public PrimitiveMsg(){}
	public PrimitiveMsg(T data){this.data = data;}
}
