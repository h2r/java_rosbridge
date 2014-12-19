package ros;

import java.util.Map;

/**
 * This is a delegate interface for handling ros topic subscriptions. The {@link #receive(java.util.Map, String)}
 * is called every time the topic with which this delegate is associated has new message published.
 * @author James MacGlashan.
 */
public interface RosListenDelegate {

	/**
	 * Receives a new publish to a subscribed topic
	 * @param data the Map representation of the JSON object.
	 * @param stringRep the string representation of the JSON object.
	 */
	public void receive(Map<String, Object> data, String stringRep);

}
