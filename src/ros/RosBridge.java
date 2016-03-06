package ros;


import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.eclipse.jetty.websocket.api.Session;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketClose;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketConnect;
import org.eclipse.jetty.websocket.api.annotations.OnWebSocketMessage;
import org.eclipse.jetty.websocket.api.annotations.WebSocket;
import org.eclipse.jetty.websocket.client.ClientUpgradeRequest;
import org.eclipse.jetty.websocket.client.WebSocketClient;

import java.io.IOException;
import java.io.StringWriter;
import java.net.URI;
import java.util.*;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

/**
 *
 * A socket for connecting to ros bridge that accepts subscribe and publish commands.
 * Subscribing to a topic using the {@link #subscribe(String, String, RosListenDelegate)}} method
 * requires a provided {@link ros.RosListenDelegate} to be provided
 * which will be informed every time this socket receives a message from a publish
 * to the subscribed topic. The message type in this method may be left null to perform
 * type inference, but subscription will fail with Rosbridge when the type is null IF the topic
 * does not already exist. You may also use the {@link #subscribe(String, String, RosListenDelegate, int, int)}
 * method to set a queue size and throttle rate. For fast data, you should set the throttle rate and queue size to 1,
 * or increasing lag may occur.
 * <br/>
 * Publishing is also supported with the {@link #publish(String, String, Object)} method, but you should
 * consider using the {@link ros.Publisher} class wrapper for streamlining publishing.
 * <br/>
 * To create and connect to rosbridge, use the {@link #createConnection(String)} method.
 * Note that rosbridge by default uses port 9090. An example URI to provide as a parameter is: ws://localhost:9090
 * @author James MacGlashan.
 */
@WebSocket(maxTextMessageSize = 64 * 1024)
public class RosBridge {

	protected final CountDownLatch closeLatch;
	protected Session session;

	protected Map<String, RosBridgeSubscriber> listeners = new HashMap<String, RosBridgeSubscriber>();
	protected Set<String> publishedTopics = new HashSet<String>();

	protected boolean hasConnected = false;

	protected boolean printMessagesAsReceived = false;


	/**
	 * Creates a connection to the ROS Bridge websocket server located at rosBridgeURI.
	 * Note that it is recommend that you call the {@link #waitForConnection()} method
	 * before publishing or subscribing.
	 * @param rosBridgeURI the URI to the ROS Bridge websocket server. Note that ROS Bridge by default uses port 9090. An example URI is: ws://localhost:9090
	 * @return the ROS Bridge socket that is connected to the indicated server.
	 */
	public static RosBridge createConnection(String rosBridgeURI){

		WebSocketClient client = new WebSocketClient();
		final RosBridge socket = new RosBridge();
		try {
			client.start();
			URI echoUri = new URI(rosBridgeURI);
			ClientUpgradeRequest request = new ClientUpgradeRequest();
			client.connect(socket, echoUri, request);
			System.out.printf("Connecting to : %s%n", echoUri);

		} catch (Throwable t) {
			t.printStackTrace();
		}

		return socket;

	}


	public RosBridge(){
		this.closeLatch = new CountDownLatch(1);
	}


	/**
	 * Blocks execution until a connection to the ros bridge server is established.
	 */
	public void waitForConnection(){

		if(this.hasConnected){
			return; //done
		}

		synchronized(this){
			while(!this.hasConnected){
				try {
					this.wait();
				} catch(InterruptedException e) {
					e.printStackTrace();
				}
			}
		}

	}


	/**
	 * Indicates whether the connection has been made
	 * @return a boolean indicating whether the connection has been made
	 */
	public boolean hasConnected(){
		return this.hasConnected;
	}


	/**
	 * Returns whether ROSBridge will print all ROSBridge messages as they are received to the command line.
	 * @return if true, then ROSBridge will print all ROSBridge messages as they are received to the command line. Otherwise is silent.
	 */
	public boolean printMessagesAsReceived() {
		return printMessagesAsReceived;
	}

	/**
	 * Sets whether ROSBridge should print all ROSBridge messages as they are received to the command.
	 * @param printMessagesAsReceived if true, then ROSBridge will print all ROSBridge messages as they are received to the command line. Otherwise is silent.
	 */
	public void setPrintMessagesAsReceived(boolean printMessagesAsReceived) {
		this.printMessagesAsReceived = printMessagesAsReceived;
	}

	/**
	 * Use this to close the connection
	 * @param duration the time in some units until closing.
	 * @param unit the unit of time in which duration is measured.
	 * @return the result of the {@link java.util.concurrent.CountDownLatch#await()} method.
	 * @throws InterruptedException
	 */
	public boolean awaitClose(int duration, TimeUnit unit) throws InterruptedException {
		return this.closeLatch.await(duration, unit);
	}

	@OnWebSocketClose
	public void onClose(int statusCode, String reason) {
		System.out.printf("Connection closed: %d - %s%n", statusCode, reason);
		this.session = null;
		this.closeLatch.countDown();
	}

	@OnWebSocketConnect
	public void onConnect(Session session) {
		System.out.printf("Got connect for ros: %s%n", session);
		this.session = session;
		this.hasConnected = true;
		synchronized(this) {
			this.notifyAll();
		}

	}

	@OnWebSocketMessage
	public void onMessage(String msg) {

		if(this.printMessagesAsReceived){
			System.out.println(msg);
		}

		ObjectMapper mapper = new ObjectMapper();
		JsonNode node = null;
		try {
			node = mapper.readTree(msg);
			if(node.has("op")){
				String op = node.get("op").asText();
				if(op.equals("publish")){
					String topic = node.get("topic").asText();
					RosBridgeSubscriber subscriber = this.listeners.get(topic);
					if(subscriber != null){
						subscriber.receive(node, msg);
					}
				}
			}
		} catch(IOException e) {
			System.out.println("Could not parse ROSBridge web socket message into JSON data");
			e.printStackTrace();
		}



	}



	/**
	 * Subscribes to a ros topic. New publish results will be reported to the provided delegate.
	 * If message type is null, then the type will be inferred. When type is null, if a topic
	 * does not already exist, subscribe will fail.
	 * @param topic the to subscribe to
	 * @param type the message type of the topic. Pass null for type inference.
	 * @param delegate the delegate that receives updates to the topic
	 */
	public void subscribe(String topic, String type, RosListenDelegate delegate){
		//already have a subscription, so just update delegate
		if(this.listeners.containsKey(topic)){
			this.listeners.get(topic).addDelegate(delegate);
			return;
		}

		//otherwise setup the subscription and delegate
		this.listeners.put(topic, new RosBridgeSubscriber(delegate));

		String subMsg = null;
		if(type != null) {
			subMsg = "{" +
					"\"op\": \"subscribe\",\n" +
					"\"topic\": \"" + topic + "\",\n" +
					"\"type\": \"" + type + "\"\n" +
					"}";
		}
		else{
			subMsg = "{" +
					"\"op\": \"subscribe\",\n" +
					"\"topic\": \"" + topic + "\"\n" +
					"}";
		}

		Future<Void> fut;
		try{
			fut = session.getRemote().sendStringByFuture(subMsg);
			fut.get(2, TimeUnit.SECONDS);
		}catch (Throwable t){
			System.out.println("Error in setting up subscription to " + topic + " with message type: " + type);
			t.printStackTrace();
		}

	}


	/**
	 * Subscribes to a ros topic. New publish results will be reported to the provided delegate.
	 * If message type is null, then the type will be inferred. When type is null, if a topic
	 * does not already exist, subscribe will fail.
	 * @param topic the to subscribe to
	 * @param type the message type of the topic. Pass null for type inference.
	 * @param delegate the delegate that receives updates to the topic
	 * @param throttleRate the minimum amount of time (in ms) that must elapse between messages being sent from the server
	 * @param queueLength the size of the queue to buffer messages. Messages are buffered as a result of the throttle_rate.
	 */
	public void subscribe(String topic, String type, RosListenDelegate delegate, int throttleRate, int queueLength){

		//already have a subscription, so just update delegate
		if(this.listeners.containsKey(topic)){
			this.listeners.get(topic).addDelegate(delegate);
			return;
		}

		//otherwise setup the subscription and delegate
		this.listeners.put(topic, new RosBridgeSubscriber(delegate));

		String subMsg = null;
		if(topic != null){
			subMsg = "{" +
				"\"op\": \"subscribe\",\n" +
				"\"topic\": \"" + topic + "\",\n" +
				"\"type\": \"" + type + "\",\n" +
				"\"throttle_rate\": " + throttleRate + ",\n" +
				"\"queue_length\": " + queueLength + "\n" +
				"}";
		}
		else{
			subMsg = "{" +
				"\"op\": \"subscribe\",\n" +
				"\"topic\": \"" + topic + "\",\n" +
				"\"throttle_rate\": " + throttleRate + ",\n" +
				"\"queue_length\": " + queueLength + "\n" +
				"}";
		}

		Future<Void> fut;
		try{
			fut = session.getRemote().sendStringByFuture(subMsg);
			fut.get(2, TimeUnit.SECONDS);
		}catch (Throwable t){
			System.out.println("Error in setting up subscription to " + topic + " with message type: " + type);
			t.printStackTrace();
		}

	}


	/**
	 * Stops a {@link RosListenDelegate} from receiving messages from Rosbridge.
	 * @param topic the topic on which the listener subscribed.
	 * @param delegate the delegate to remove.
	 */
	public void removeListener(String topic, RosListenDelegate delegate){

		RosBridgeSubscriber subscriber = this.listeners.get(topic);
		if(subscriber != null){
			subscriber.removeDelegate(delegate);
		}

	}


	/**
	 * Advertises that this object will be publishing to a ROS topic.
	 * @param topic the topic to which this object will be publishing.
	 * @param type the ROS message type of the topic.
	 */
	public void advertise(String topic, String type){

		if(!this.publishedTopics.contains(topic)){

			//then start advertising first
			String adMsg = "{" +
					"\"op\": \"advertise\",\n" +
					"\"topic\": \"" + topic + "\",\n" +
					"\"type\": \"" + type + "\"\n" +
					"}";

			Future<Void> fut;
			try{
				fut = session.getRemote().sendStringByFuture(adMsg);
				fut.get(2, TimeUnit.SECONDS);
				this.publishedTopics.add(topic);
			}catch (Throwable t){
				System.out.println("Error in setting up advertisement to " + topic + " with message type: " + type);
				t.printStackTrace();
			}

		}

	}


	/**
	 * Publishes to a topic. If the topic has not already been advertised on ros, it will automatically do so.
	 * @param topic the topic to publish to
	 * @param type the message type of the topic
	 * @param msg should be a {@link java.util.Map} or a Java Bean, specifying the ROS message
	 */
	public void publish(String topic, String type, Object msg){

		this.advertise(topic, type);

		Map<String, Object> jsonMsg = new HashMap<String, java.lang.Object>();
		jsonMsg.put("op", "publish");
		jsonMsg.put("topic", topic);
		jsonMsg.put("type", type);
		jsonMsg.put("msg", msg);

		JsonFactory jsonFactory = new JsonFactory();
		StringWriter writer = new StringWriter();
		JsonGenerator jsonGenerator;
		ObjectMapper objectMapper = new ObjectMapper();

		try {
			jsonGenerator = jsonFactory.createGenerator(writer);
			objectMapper.writeValue(jsonGenerator, jsonMsg);
		} catch(Exception e){
			System.out.println("Error");
		}

		String jsonMsgString = writer.toString();
		Future<Void> fut;
		try{
			fut = session.getRemote().sendStringByFuture(jsonMsgString);
			fut.get(2, TimeUnit.SECONDS);
		}catch (Throwable t){
			System.out.println("Error publishing to " + topic + " with message type: " + type);
			t.printStackTrace();
		}

	}


	/**
	 * Publishes to a topic with a ros message represented in its JSON string form.
	 * If the topic has not already been advertised on ros, it will automatically do so.
	 * @param topic the topic to publish to
	 * @param type the message type of the topic
	 * @param jsonMsg the JSON string of the ROS message.
	 */
	public void publishJsonMsg(String topic, String type, String jsonMsg){

		this.advertise(topic, type);

		String fullMsg = "{\"op\": \"publish\", \"topic\": \"" + topic + "\", \"type\": \"" + type + "\", " +
				"\"msg\": " + jsonMsg + "}";


		Future<Void> fut;
		try{
			fut = session.getRemote().sendStringByFuture(fullMsg);
			fut.get(2, TimeUnit.SECONDS);
		}catch (Throwable t){
			System.out.println("Error publishing to " + topic + " with message type: " + type);
			t.printStackTrace();
		}

	}


	/**
	 * Sends the provided fully specified message to the ROS Bridge server. Since the RosBridge server
	 * expects JSON messages, the string message should probably be in JSON format and adhere to the R
	 * Rosbridge protocol, but this method will send whatever raw string you provide.
	 * @param message the message to send to Rosbridge.
	 */
	public void sendRawMessage(String message){

		Future<Void> fut;
		try{
			fut = session.getRemote().sendStringByFuture(message);
			fut.get(2, TimeUnit.SECONDS);
		}catch (Throwable t){
			System.out.println("Error sending raw message to RosBridge server: " + message);
			t.printStackTrace();
		}

	}

	/**
	 * Attempts to turn the the provided object into a JSON message and send it to the ROSBridge server.
	 * If the object does not satisfy the Rosbridge protocol, it may have no affect.
	 * @param o the object to turn into a JSON message and send.
	 */
	public void formatAndSend(Object o){

		JsonFactory jsonFactory = new JsonFactory();
		StringWriter writer = new StringWriter();
		JsonGenerator jsonGenerator;
		ObjectMapper objectMapper = new ObjectMapper();

		try {
			jsonGenerator = jsonFactory.createGenerator(writer);
			objectMapper.writeValue(jsonGenerator, o);
		} catch(Exception e){
			System.out.println("Error parsing object into a JSON message.");
		}

		String jsonMsgString = writer.toString();
		Future<Void> fut;
		try{
			fut = session.getRemote().sendStringByFuture(jsonMsgString);
			fut.get(2, TimeUnit.SECONDS);
		}catch (Throwable t){
			System.out.println("Error sending message to RosBridge server: " + jsonMsgString);
			t.printStackTrace();
		}

	}


	/**
	 * Class for managing all the listeners that have subscribed to a topic on Rosbridge.
	 * Maintains a list of {@link RosListenDelegate} objects and informs them all
	 * when a message has been received from Rosbridge.
	 */
	public static class RosBridgeSubscriber{

		List<RosListenDelegate> delegates = new ArrayList<RosListenDelegate>();

		public RosBridgeSubscriber() {
		}

		/**
		 * Initializes and adds all the input delegates to receive messages.
		 * @param delegates the delegates to receive messages.
		 */
		public RosBridgeSubscriber(RosListenDelegate...delegates) {
			for(RosListenDelegate delegate : delegates){
				this.delegates.add(delegate);
			}
		}

		/**
		 * Adds a delegate to receive messages from Rosbridge.
		 * @param delegate a delegate to receive messages from Rosbridge.
		 */
		public void addDelegate(RosListenDelegate delegate){
			this.delegates.add(delegate);
		}


		/**
		 * Removes a delegate from receiving messages from Rosbridge
		 * @param delegate the delegate to stop receiving messages.
		 */
		public void removeDelegate(RosListenDelegate delegate){
			this.delegates.remove(delegate);
		}

		/**
		 * Receives a new published message to a subscribed topic and informs all listeners.
		 * @param data the {@link com.fasterxml.jackson.databind.JsonNode} containing the JSON data received.
		 * @param stringRep the string representation of the JSON object.
		 */
		public void receive(JsonNode data, String stringRep){
			for(RosListenDelegate delegate : delegates){
				delegate.receive(data, stringRep);
			}
		}

	}

}
