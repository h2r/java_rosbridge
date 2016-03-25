package ros;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.util.HashMap;
import java.util.Map;

/**
 * A Subscription request builder. Supports all Rosbridge protocol fields for
 * a subscription request except png compression. Requires a topic to be set,
 * with the rest of the optional values set with setter methods that return
 * the this object so that you can chain settters on a single line. Use
 * the static {@link #generate(String)} to start the sequence. If a value is set
 * to null, it will be removed from the message.
 * <p>
 * When everything is set, the JSON message can be retrieved with the {@link #generateJsonString()}
 * @author James MacGlashan.
 */
public class SubscriptionRequestMsg {

	protected Map<String, Object> keyValues = new HashMap<String, Object>(7);

	public static SubscriptionRequestMsg generate(String topic){
		return new SubscriptionRequestMsg(topic);
	}

	public SubscriptionRequestMsg(String topic){
		if(topic == null){
			throw new RuntimeException("ROS topic cannot be null in subscription request.");
		}
		keyValues.put("op", "subscribe");
		keyValues.put("topic", topic);
	}

	public SubscriptionRequestMsg setTopic(String topic){
		this.setKeyValue("topic", topic);
		return this;
	}

	public SubscriptionRequestMsg setType(String type){
		this.setKeyValue("type", type);
		return this;
	}

	public SubscriptionRequestMsg setThrottleRate(Integer throttleRate){
		this.setKeyValue("throttle_rate", throttleRate);
		return this;
	}

	public SubscriptionRequestMsg setQueueLength(Integer queueLength){
		this.setKeyValue("queue_length", queueLength);
		return this;
	}

	public SubscriptionRequestMsg setFragmentSize(Integer fragmentSize){
		this.setKeyValue("fragment_size", fragmentSize);
		return this;
	}


	public SubscriptionRequestMsg setId(String id){
		this.setKeyValue("id", id);
		return this;
	}

	public String getTopic(){
		return (String)this.keyValues.get("topic");
	}

	public String getType(){
		return (String)this.keyValues.get("type");
	}

	public Integer getThrottleRate(){
		return (Integer)this.keyValues.get("throttle_rate");
	}

	public Integer getQueueLength(){
		return (Integer)this.keyValues.get("queue_length");
	}

	public Integer getFragmentSize(){
		return (Integer)this.keyValues.get("fragment_size");
	}


	public String getId(){
		return (String)this.keyValues.get("id");
	}

	/**
	 * Generates the JSON string for this subscription request.
	 * @return the JSON string for this subscription request.
	 */
	public String generateJsonString(){
		ObjectMapper mapper = new ObjectMapper();
		String jsonString = null;
		try {
			jsonString = mapper.writeValueAsString(this.keyValues);
		} catch(JsonProcessingException e) {
			e.printStackTrace();
		}

		return jsonString;
	}


	protected void setKeyValue(String key, Object value){
		if(value == null){
			this.keyValues.remove(key);
		}
		else{
			this.keyValues.put(key, value);
		}
	}

}
