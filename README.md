java_rosbridge
==============

A simple library using Jetty 9 to connect Java code to a [ROS Bridge server](http://wiki.ros.org/rosbridge_suite/). This library supports publishing and subscribing with different topic delegates and makes using Java code with ROS very trivial (we argue that it is easier than using ROSJava which has a number of complications and complexities).

##Linking
java_rosbridge is indexed on Maven Central, so if you want to merely use it, all you need to do is include in the `<dependencies>` section of your project's pom.xml file:
```
<dependency>
  <groupId>edu.brown.cs.burlap</groupId>
  <artifactId>java_rosbridge</artifactId>
  <version>2.0.1</version>
</dependency>
```
and it will automatically be downloaded. Alternatively, you may compile and install the code directly (or modify as needed), as described in the compiling section of this readme.

## Compiling

Compiling and usage is now performed with Maven. If you would like to compile with ant, use the ant branch of this repo. However, going forward, updated version of java_rosbridge will require Maven. 

If you do not have Maven installed on your system, get it from https://maven.apache.org/download.cgi

Compile with:

```
mvn compile
```

Create the target jar and Java doc with

```
mvn package
```

Install into your local repo with

```
mvn install
```

Have other projects use java_rosbridge by adding the following to the projects pom.xml `<dependencies>` section:

```
<dependency>
  <groupId>edu.brown.cs.burlap</groupId>
  <artifactId>java_rosbridge</artifactId>
  <version>2.0.1</version>
</dependency>
```

## Using the code

Here we will provide a description of the library code, but you should consult the Java doc for more detailed information.

### Websockets test code

The first thing to note is the `tests` package shows some very basic test code that you can run. The `SimpleEchoClient` class runs a very simple example of websockets running using Jetty; it does not have anything specifically to do with ROS and is a way of sanity checking the websocket code. It makes reference to the `SimpleEchoSocket` class for handling the communication. All this code was provided by the Jetty tutorial. The `echo.websocket.org` server to which the code connects is a simple websocket server for testing websockets (also specified for use in the Jetty tutorial). When you run it, you should get a long connect debug message printed to the terminal and the following which are the messages received from the `echo.websocket.org` server:

```
Got msg: Hello
Got msg: Thanks for the conversation.
Connection closed: 1000 - I'm done
```


### ROS Bridge test code

The `RosTest` code in the `tests` package provides a demonstration of connecting to ROS Bridge and publishing the string `hello from java` to the ROS topic `/java_to_ros` every 500 ms. The code will also subscribe to the `std_msgs/String` topic `/ros_to_java`. For convenience, we have also posted the code below in this readme.

```
import com.fasterxml.jackson.databind.JsonNode;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;

/**
 * Example of connecting to rosbridge with publish/subscribe messages. Takes one argument:
 * the rosbridge websocket URI; for example: ws://localhost:9090.
 * @author James MacGlashan.
 */
public class RosTest {

	public static void main(String[] args) {

		if(args.length != 1){
			System.out.println("Need the rosbridge websocket URI provided as argument. For example:\n\tws://localhost:9090");
			System.exit(0);
		}

		RosBridge bridge = new RosBridge();
		bridge.connect(args[0], true);

		bridge.subscribe(SubscriptionRequestMsg.generate("/ros_to_java")
					.setType("std_msgs/String")
					.setThrottleRate(1)
					.setQueueLength(1),
				new RosListenDelegate() {
					@Override
					public void receive(JsonNode data, String stringRep) {
						MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(PrimitiveMsg.class);
						PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
						System.out.println(msg.data);
					}
				}
		);



		Publisher pub = new Publisher("/java_to_ros", "std_msgs/String", bridge);

		for(int i = 0; i < 100; i++) {
			pub.publish(new PrimitiveMsg<String>("hello from java " + i));
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

}
```


This code takes as a command line argument the URI to the ROS Bridge server. Note that ROS Bridge by default runs on port 9090. An example URI is `ws://localhost:9090`. Obviously, you should launch ROS Bridge on the server before running this example code. (That is, before running the `RosTest` class, run `roslaunch rosbridge_server rosbridge_websocket.launch` on the ROS server. For more information on getting and starting ROS Bridge on your ROS server, see [here](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge).) To observe the Java publishing over ROS Bridge on the actual ROS Bridge server, run the ROS command

```
rostopic echo /java_to_ros
```
on the ROS server. You should find it outputting the published message `hello from java i` where `i` is an int representing which publish it is. 

To observe the subscrption code in action, on ROS you will want to make sure you're publishing a `std_msgs/String` message to the topic `ros_to_java`. To do that, try using the ROS command
```
rostopic pub ros_to_java std_msgs/String "hello from ros"
```

If you do so, then when you run the Java code, you will see that message printed to the terminal.


### Primary Library Classes
The primary library classes consist of `RosBridge` for setting up the connection to ROS Bridge, `Publisher` for publishing messages to ROS over ROS Bridge, and `RosListenDelegate` for managing call backs when you subscribe to a ROS topic.

####RosBridge
As the `RosTest` code illustrates, the main Java object you work with is an instance of the `RosBridge` class that is defined in the `ros` package. `RosBridge` objects are created using the static method `createConnection(String rosBridgeURI)` (which takes the URI of the ROS Bridge server). Note that after creating a connection it is a good idea to call the `waitForConnection()` method, which will block the client thread until the connection with the ROS Bridge server has been established. A client Java object interacts with a `RosBridge` object with publish commands and subscription requests, each of which are discussed below.

####Publish
Publish commands can be given with the `publish(String topic, String type, Object msg)` method of the `RosBridge` class, which automatically advertises the publish topic if it has not yet already been advertised. A more streamlined and ROS-like way to handle publishing (and what is demonstrated in the `RosTest` code) is to create a `Publisher` object instance (also defined in the `ros` package), which maintains the topic name and ROS topic message type so that you only need to pass its `publish(Object)` method the message you wish to publish. 

The message object you pass the publish method can either be a [Java Bean](https://en.wikipedia.org/wiki/JavaBeans) whose data members correspond to the ROS message structure, or a Java Map data structure whose keys are strings representing the names of the ROS message fields and values are the corresponding data type or another nested Map. (Note that a [Java Bean](https://en.wikipedia.org/wiki/JavaBeans) is a Java object with a default constructor and either public data members or data members that have setter and getter methods following standard Java method name conventions.) 

`java_rosbridge` includes some Java Bean data structures for common ROS messages in the `msgs` package. For example, the `PrimitiveMsg` is a Java Bean that can be used for many of the primitive ROS message types found in the `std_msgs` ROS package by setting its generic type to the appropriate primitive. For example, `PrimitiveMsg<String>` can be used for the `std_msgs/String` message. Alternatively, if you wanted to recreate the `std_msgs/String` message with a Java Map, the Map should contain one key-value entry of `data: stringValue`, where `stringValue` is whatever the ROS `std_msgs/String` data field value is.

####Subcribe
You can subscribe to ROS topics via the `subscribe(SubscriptionRequestMsg request, RosListenDelegate delegate)` method (other variants of this method also exist). The SubscriptionRequestMsg object is a class for easily specifying the various optional fields you wish to include in the subscription method to manage the network details. The optional fields are message type, throttle_rate, queue_length, and fragment_size. This library currently does not support png compression. The other method argument is a `RosListenDelegate` (an interface defined in the `ros` package) which is a callback object that is passed the data sent from ROS Bridge for the specific topic to which you subscribed. You will need to implement your own `RosListenDelegate` object to process the received data, similar to how in normal Python ROS you implement a callback function to process subscriptions.

**Recommendation**: if you are subscribing to a high frequency topic, you should set the subcribed `throttleRate` and `queueLength` to 1 (or some other small values), otherwise your received data will increasingly lag behind the current real time values.

### Implementing a RosListenDelegate

A `RosListenDelegate` is an interface to define the callback function for subscribed topic messages sent over ROSBridge. It requires that you implement the method `receive(JsonNode data, String stringRep)`. The two parameters of this message present the ROSBridge message in two different formats. The latter, `stringRep`, is the string representation of the ROSBridge message which allows you to do anything you want with the raw data. The former, `data`, is a [JsonNode](http://fasterxml.github.io/jackson-databind/javadoc/2.2.0/com/fasterxml/jackson/databind/JsonNode.html) of the data sent over ROSBridge. `data` has four top-level JSON fields:

- `op`: which kind of messag operation it was; should always be "publish" (ROSBridge is passing a published message)
- `topic`: to which topic the message was published
- `type`: the ROS message type of the topic
- `msg`: the provided ros message in JSON format

The `msg` field is the primary field of interest since it contains the actual ROS message. The simplest way to parse a ROS message is to unpack it into a corresponding Java Bean for the ROS message type. The `MessageUnpacker` class can help you do that. As shown in the example code, a `MessageUnpacker` takes a generic specifying the type of [Java Bean](https://en.wikipedia.org/wiki/JavaBeans) into which you're going to unpack the message and its constructor also requries you to give the Java .class reference. Then you can use its `unpackRosMessage(JsonNode)` method, providing it the `JsonNode` given to the `RosListenDelegate` `receive` method, as shown in the example code. (Note that you do not need to unpack the msg field first, the `MessageUnpacker` method will automatically do that for you.)

Alternatively, if you do not have a Java Bean class into which you can trivially unpack the ROS message, you can also iterate through the fields of the `JsonNode` manually. Working with JSON data using the `JsonNode` data structure is very easy because it has getter methods for JSON fields and elements in JSON arrays. For example, if the ROS message is a `geometry_msgs/Twist.msg` message, and you want the linear x component, you can retreive it with the code:

`double x = data.get("msg").get("linear").get("x").asDouble();`.

If a field's value is an array, it still is returned as a `JsonNode`; however, `JsonNode` has convenient methods for working with it. To get the size of the array use `node.size()` where `node` is the current `JsonNode` element you're using. To get an element within it (also returned as a `JsonNode` for recrusion of non-primitives), use `node.get(i)`, where `i` is the index into the array. See the [JsonNode Java documentation](http://fasterxml.github.io/jackson-databind/javadoc/2.2.0/com/fasterxml/jackson/databind/JsonNode.html) for more information on working with a `JsonNode`.


### Large Message Sizes

Some messages you may wish to recieve/send from/to ROS are very large, such as video feeds and they may be larger than the default buffer size that Jetty's websocket uses. One solution to this limitiation is to use fragementation in your subscription request. However, you may also increase your web socket message size buffer so that you do not need to use fragementation. Since we use Jetty for managing the websocket network, increasing the web socket buffer size is accomplished by subclassing the RosBridge object and annotating it with `@Websocket` with a larger parameter value for the message size. The subclass does not need to override or implement any methods; we extend the class purely to annotate it with a custom web socket buffer size. For example:

```
@WebSocket(maxTextMessageSize = 500 * 1024) public class BigMessageRosBridge extends RosBridge{}
```

Then you should instantiate and use your subclass instead of the top-level RosBridge.
