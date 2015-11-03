package ros.tools;

import ros.Publisher;
import ros.RosBridge;

import java.util.Timer;
import java.util.TimerTask;

/**
 * This class is a tool for causing a ROS message to be periodically published at a given rate. To start periodic publishing
 * use the {@link #beginPublishing(int)} or {@link #beginPublishing(Object, int)} methods. Note that subsequent calls
 * to these method will automatically cancel the previous periodic publishing and start again at the newly specified rate.
 * To cancel periodic publishing, use the {@link #cancelPublishing()} method. You can also change the message that is
 * being published without stopping or restarting the publishing by using the {@link #setMsg(Object)} method.
 * @author James MacGlashan.
 */
public class PeriodicPublisher {


	/**
	 * The {@link ros.Publisher} to which publish calls are made
	 */
	protected Publisher pub;

	/**
	 * The ROS message that will be periodically published
	 */
	protected volatile Object msg = null;

	/**
	 * The period at which this object is publishing; -1 if it is not currently publishing.
	 */
	protected int period = -1;

	/**
	 * The timer task that calls the publish. Null if this object is not periodically publishing.
	 */
	protected PublisherTimerTask timerTask;

	/**
	 * Initializes with a {@link ros.Publisher} for this action using the specified topic, message type, and {@link ros.RosBridge} connection.
	 * @param topic the ros topic to which messages will be published
	 * @param msgType the ros message type of th target topic
	 * @param rosBridge the {@link ros.RosBridge} connection to use.
	 */
	public PeriodicPublisher(String topic, String msgType, RosBridge rosBridge){
		this.pub = new Publisher(topic, msgType, rosBridge);
	}

	/**
	 * Initializes with a {@link ros.Publisher} for this action using the specified topic, message type, and {@link ros.RosBridge} connection.
	 * @param topic the ros topic to which messages will be published
	 * @param msgType the ros message type of th target topic
	 * @param rosBridge the {@link ros.RosBridge} connection to use.
	 * @param msg the ROS message to periodically publish when publishing begins
	 */
	public PeriodicPublisher(String topic, String msgType, RosBridge rosBridge, Object msg){
		this.pub = new Publisher(topic, msgType, rosBridge);
	}

	/**
	 * Initializes with a {@link ros.Publisher} for publishing action messages to ROS.
	 * @param pub the {@link ros.Publisher} to which to publish
	 */
	public PeriodicPublisher(Publisher pub){
		this.pub = pub;
	}

	/**
	 * Initializes with a {@link ros.Publisher} for publishing action messages to ROS.
	 * @param pub the {@link ros.Publisher} to which to publish
	 * @param msg the ROS message to periodically publish when publishing begins
	 */
	public PeriodicPublisher(Publisher pub, Object msg) {
		this.pub = pub;
		this.msg = msg;
	}

	public Publisher getPub() {
		return pub;
	}

	public void setPub(Publisher pub) {
		this.pub = pub;
	}

	/**
	 * Returns the ROS message that is being or will be periodically published.
	 * @return the ROS message that is being or will be periodically published.
	 */
	public Object getMsg() {
		return msg;
	}

	/**
	 * Sets the ROS message that will be periodically published. Note that if the provided message is null and this
	 * object is already periodically publishing, a runtime error will be thrown.
	 * @param msg the ROS message that will be published.
	 */
	public void setMsg(Object msg) {
		if(msg == null && this.timerTask != null){
			System.out.println("PeriodicPublisher is not setting message because new message is null and publisher is currently publishing.");
		}
		this.msg = msg;
	}

	/**
	 * The delay in milliseconds between subsequent periodic publishes. Returns -1 if this object is not currently periodically publishing.
	 * @return The delay in milliseconds between subsequent publishes. Returns -1 if this object is not currently periodically publishing.
	 */
	public int getPublishingPeriod(){
		return this.period;
	}

	/**
	 * Stops this object from periodically publishing, or does nothing if it is not currently periodically publishing.
	 */
	public void cancelPublishing(){
		if(this.timerTask != null) {
			this.timerTask.cancel();
			this.timerTask = null;
			this.period = -1;
		}
	}

	/**
	 * Indicates whether this object is currently periodically publishing.
	 * @return true if this object is periodically publishing; false if it is not.
	 */
	public boolean isPublishing(){
		return this.timerTask != null;
	}


	/**
	 * Causes this object to begin periodically publishing at the specified rate. If this object is
	 * already publishing then the previous publishing rate is canceled and started at the new rate.
	 * If the current message to
	 * publish is not set, then a runtime exception will be thrown. The message can be set either with the
	 * {@link #setMsg(Object)} method or by using the alternative {@link #beginPublishing(Object, int)}
	 * method that takes the message to publish.
	 * @param period the time in milliseconds between publishes
	 */
	public void beginPublishing(int period){
		if(this.msg == null){
			throw new RuntimeException("Cannot begin publishing because the message to publish is unset. " +
					"Use the setMsg method or beginPublishing(Object msg, int period)");
		}
		if(this.timerTask != null){
			this.timerTask.cancel();
		}
		this.timerTask = new PublisherTimerTask();
		Timer timer = new Timer();
		timer.schedule(this.timerTask, 0, period);
	}


	/**
	 * Causes this object to begin periodically publishing the specified message at the specified rate.
	 * If this object is already publishing then the previous publishing rate is canceled and started at the new rate.
	 * Note that if the provided message is null then a runtime exception will be thrown.
	 * @param msg the ROS message to publish
	 * @param period the time in milliseconds between publishes
	 */
	public void beginPublishing(Object msg, int period){
		this.msg = msg;
		this.beginPublishing(period);
	}


	/**
	 * The class that is called by Java's {@link java.util.Timer} and invokes the actual
	 * publish call.
	 */
	protected class PublisherTimerTask extends TimerTask{

		@Override
		public void run() {
			pub.publish(msg);
		}
	}

}
