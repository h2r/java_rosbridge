package ros.msgs.std_msgs;

/**
 * @author James MacGlashan.
 */
public class Header {

	public int seq;
	public Time stamp;
	public String frame_id;

	public Header() {
	}

	public Header(int seq, Time stamp, String frame_id) {
		this.seq = seq;
		this.stamp = stamp;
		this.frame_id = frame_id;
	}

}
