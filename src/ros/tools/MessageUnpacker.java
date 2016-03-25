package ros.tools;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * This is a helper class that may be used for taking a JSON message returned from Ros bridge, and unpacking
 * the actual ros message inside it into a Java bean object. (A Java bean is a Java object that has a default
 * constructor and getter and has either public data members or getter and setter methods for the data members
 * using standard Java naming conventions.)
 * <p>
 * To use this class, type the generic to the Java Bean class to which the message will be unpacked and provide
 * the constructor the class of it as well. For example:<p>
 * <code>MessageUnpacker&lt;Twist&gt; unpacker = new MessageUnpacker&lt;Twist&gt;(Twist.class);</code>. Then provide the
 * {@link #unpackRosMessage(com.fasterxml.jackson.databind.JsonNode)} method the {@link com.fasterxml.jackson.databind.JsonNode}
 * provided to a {@link ros.RosListenDelegate} to unpack the "msg" field into the Java Bean. This will also work
 * with Java Beans with generics. For example, you can do
 * <code>MessageUnpacker&lt;PrimitiveMsg&lt;String&gt;&gt; unpacker = new MessageUnpacker&lt;PrimitiveMsg&lt;String&gt;&gt;(PrimitiveMsg.class);</code>.
 *
 * @author James MacGlashan.
 */
public class MessageUnpacker <T> {
	protected ObjectMapper mapper = new ObjectMapper();
	protected Class<?> javaClass;

	/**
	 * Constructor.
	 * @param javaClass a {@link java.lang.Class} specifying the class of the Java bean into which
	 *                  a ros message will be unpacked.
	 */
	public MessageUnpacker(Class<?> javaClass) {
		this.javaClass = javaClass;
	}

	/**
	 * Unpacks a ros message into the appropriate Java bean from the JSON message returned by
	 * ros bridge. Note that the Ros bridge JSON message contains header information and the actual
	 * ROS message is stored in the 'msg' field in the JSON message. Therefore, this method should be provided the
	 * {@link com.fasterxml.jackson.databind.JsonNode} that is given to a {@link ros.RosListenDelegate}
	 * {@link ros.RosListenDelegate#receive(com.fasterxml.jackson.databind.JsonNode, String)} method.
	 * If the provided {@link com.fasterxml.jackson.databind.JsonNode} argument of this method
	 * does not have a "msg" field, then it will attempt to unpack from the whole JSON node.
	 * @param rosBridgeMessage the {@link com.fasterxml.jackson.databind.JsonNode} from RosBridge.
	 * @return the unpacked Java Bean of the ROS message.
	 */
	@SuppressWarnings("unchecked")
	public T unpackRosMessage(JsonNode rosBridgeMessage){
		JsonNode rosMsgNode = rosBridgeMessage.get("msg");
		if(rosMsgNode == null){
			rosMsgNode = rosBridgeMessage;
		}
		T rosMsg = null;
		try {
			rosMsg = (T)mapper.treeToValue(rosMsgNode, javaClass);
		} catch(JsonProcessingException e) {
			e.printStackTrace();
		}
		return rosMsg;
	}
}
