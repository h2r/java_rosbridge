package ros.msgs.sensor_msgs;

import ros.msgs.std_msgs.Header;

import java.awt.image.BufferedImage;

/**
 * Implementation of ROS sensor_msgs/Image.msg:
 * <a href="http://docs.ros.org/api/sensor_msgs/html/msg/Image.html">http://docs.ros.org/api/sensor_msgs/html/msg/Image.html</a>.
 * This class can also decode the ROS Image into a Java Buffered Image for images that are encoded in either
 * bgr8, rgb8, or mono8, by using the {@link #toBufferedImage()} method.
 * @author James MacGlashan.
 */
public class Image {

	public Header header;
	public int height;
	public int width;
	public String encoding;
	public int is_bigendian;
	public int step;
	public byte[] data;

	public Image() {
	}

	public Image(Header header, int height, int width, String encoding, int is_bigendian, int step, byte[] data) {
		this.header = header;
		this.height = height;
		this.width = width;
		this.encoding = encoding;
		this.is_bigendian = is_bigendian;
		this.step = step;
		this.data = data;
	}

	/**
	 * Constructs a {@link BufferedImage} from this ROS Image, provided the encoding is either rgb8, bgr8, or mono8.
	 * If it is not one of those encodings, then a runtime exception will be thrown.
	 * @return a {@link BufferedImage} representation of this image.
	 */
	public BufferedImage toBufferedImage(){

		if(this.encoding.equals("bgr8") || this.encoding.equals("rgb8")){
			return this.toBufferedImageFromRGB8();
		}
		else if(this.encoding.equals("mono8")){
			return this.toBufferedImageFromMono8();
		}


		throw new RuntimeException("ROS Image does not currently decode " + this.encoding + ". See Java doc for support types.");
	}


	/**
	 * Constructs a {@link BufferedImage} from this ROS Image assuming the encoding is mono8
	 * @return a {@link BufferedImage} representation of this image.
	 */
	protected BufferedImage toBufferedImageFromMono8(){
		int w = this.width;
		int h = this.height;

		BufferedImage i = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < h; ++y) {
			for(int x = 0; x < w; ++x) {

				//row major
				int index = (y * w) + x;
				// combine to RGB format
				int anded = data[index++] & 0xFF;
				int rgb = anded |
						(anded << 8) |
						(anded << 16) |
						0xFF000000;

				i.setRGB(x, y, rgb);
			}
		}

		return i;
	}


	/**
	 * Constructs a {@link BufferedImage} representation from this ROS Image assuming the encoding is either rgb8 or bgr8.
	 * @return a {@link BufferedImage} representation of this image.
	 */
	protected BufferedImage toBufferedImageFromRGB8(){

		int w = this.width;
		int h = this.height;

		BufferedImage i = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < h; ++y) {
			for (int x = 0; x < w; ++x) {

				//row major, consecutive channels
				int index = (y * w * 3) + (x * 3);
				// combine to RGB format
				int rgb;
				if(this.encoding.equals("bgr8")){
					rgb = ((data[index++] & 0xFF)) |
							((data[index++] & 0xFF) << 8) |
							((data[index++] & 0xFF) << 16) |
							0xFF000000;
				}
				else if(this.encoding.equals("rgb8")){
					rgb = ((data[index++] & 0xFF) << 16) |
							((data[index++] & 0xFF) << 8) |
							((data[index++] & 0xFF)) |
							0xFF000000;
				}
				else{
					throw new RuntimeException("ROS Image toBufferedImageFromRGB8 does not decode " + this.encoding);
				}
				i.setRGB(x, y, rgb);
			}
		}

		return i;

	}

}
