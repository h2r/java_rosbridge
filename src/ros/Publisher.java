package ros;

/**
 * A wrapper class for streamlining ROS Topic publishing. Note that the {@link #advertise()} never *needs* to be explicitly
 * called. If you use the standard {@link #Publisher(String, String, RosBridge)} method, it will be automatically
 * called on construction and if you use the {@link #Publisher(String, String, RosBridge, boolean)} method
 * and set the advertiseNow flag to false, you still don't *need* to call it, because the first publish will
 * automatically make sure the topic has been advertised first.
 * <p>
 * Publish messages using the {@link #publish(Object)} method. It takes an object containing the ROS message
 * to publish. Generally, the msg should either be a Javabean, such as one of the pre-included
 * messages in the {@link ros.msgs} package that has the same field structure as the target topic type
 * or a {@link java.util.Map} object
 * representing the ROS message type structure. For
 * example, if the ROS message type is "std_msgs/String" then msg should be a {@link ros.msgs.std_msgs.PrimitiveMsg}
 * with the generic of String, or a Map object
 * with one Map key-value entry of "data: stringValue" where stringValue is whatever the "std_msgs/String"
 * data field value is.
 * @author James MacGlashan.
 */
public class Publisher {

	protected String topic;
	protected String msgType;
	protected RosBridge rosBridge;


	/**
	 * Constructs and automatically advertises publishing to this topic.
	 * @param topic the topic to which messages will be published
	 * @param msgType the ROS message type of the topic
	 * @param rosBridge the {@link ros.RosBridge} that manages ROS Bridge interactions.
	 */
	public Publisher(String topic, String msgType, RosBridge rosBridge){
		this.topic = topic;
		this.msgType = msgType;
		this.rosBridge = rosBridge;

		this.rosBridge.advertise(this.topic, this.msgType);

	}


	/**
	 * Constructs and advertises if the advertiseNow flag is set to true.
	 * @param topic the topic to which messages will be published
	 * @param msgType the ROS message type of the topic
	 * @param rosBridge the {@link ros.RosBridge} that manages ROS Bridge interactions.
	 * @param advertiseNow if true, then the topic is advertised; if false then it is not yet advertised.
	 */
	public Publisher(String topic, String msgType, RosBridge rosBridge, boolean advertiseNow){
		this.topic = topic;
		this.msgType = msgType;
		this.rosBridge = rosBridge;

		if(advertiseNow) {
			this.rosBridge.advertise(this.topic, this.msgType);
		}

	}


	/**
	 * Advertises to ROS that this topic will have messages published to it. You never
	 * *need* to call this method since publishes will always make sure it was advertised first,
	 * but gives you control if you did not have the topic advertised at constructions.
	 */
	public void advertise(){
		this.rosBridge.advertise(this.topic, this.msgType);
	}


	/**
	 * Publishes the message. If this client is not already advertising for this topic, it automatically will first.<p>
	 * Generally, the msg should either be a Javabean, such as one of the pre-included
	 * messages in the {@link ros.msgs} package that has the same field structure as the target topic type
	 * or a {@link java.util.Map} object
	 * representing the ROS message type structure. For
	 * example, if the ROS message type is "std_msgs/String" then msg should be a {@link ros.msgs.std_msgs.PrimitiveMsg}
	 * with the generic of String, or a Map object
	 * with one Map key-value entry of "data: stringValue" where stringValue is whatever the "std_msgs/String"
	 * data field value is.
	 * @param msg the message to publish.
	 */
	public void publish(Object msg){
		this.rosBridge.publish(this.topic, this.msgType, msg);
	}


	/**
	 * Publishes a ROS message specified in a JSON string.
	 * If this client is not already advertising for this topic, it automatically will first.
	 * @param jsonMsg the ROS message specified in a JSON string.
	 */
	public void publishJsonMsg(String jsonMsg){
		this.rosBridge.publishJsonMsg(this.topic, this.msgType, jsonMsg);
	}


	/**
	 * Returns the topic topic to which this object publishes.
	 * @return the topic topic to which this object publishes.
	 */
	public String getTopic() {
		return topic;
	}


	/**
	 * Returns the ROS message type of the topic to which this object publishes.
	 * @return the ROS message type of the topic to which this object publishes.
	 */
	public String getMsgType() {
		return msgType;
	}


	/**
	 * Returns the {@link ros.RosBridge} object that manages the connection to the ROS Bridge server.
	 * @return the {@link ros.RosBridge} object that manages the connection to the ROS Bridge server.
	 */
	public RosBridge getRosBridge() {
		return rosBridge;
	}


	/**
	 * Unadvertises that you are publishing to the topic.
	 */
	public void unadvertise(){
		this.rosBridge.unadvertise(this.topic);
	}
}
