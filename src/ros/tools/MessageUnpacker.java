package ros.tools;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/**
 * This is a helper class that may be used for tasking a Json message returned from Rosbridge, and unpacking
 * the actual ros message inside it into a Java bean object.
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
	 * ros bridge. Note that the Ros bridge JSON message contains header information with the actual
	 * ROS message as the 'msg' field in the JSON message. Therefore, this method should be provided the
	 * {@link com.fasterxml.jackson.databind.JsonNode} that is given to a {@link ros.RosListenDelegate}
	 * {@link ros.RosListenDelegate#receive(com.fasterxml.jackson.databind.JsonNode, String)} method.
	 * If the provided {@link com.fasterxml.jackson.databind.JsonNode} argument of this method
	 * does not have a "msg" field, then it will attempt to unpack from the whole JSON node.
	 * @param rosBridgeMessage the {@link com.fasterxml.jackson.databind.JsonNode} from RosBridge.
	 * @return the unpacked Java Bean of the ROS message.
	 */
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
