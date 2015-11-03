java_rosbridge
==============

A simple library using Jetty 9 to connect Java code to a [ROS Bridge server](http://wiki.ros.org/rosbridge_suite/). This library supports publishing and subscribing with different topic delegates and makes using Java code with ROS very trivial (we argue that it is easier than using ROSJava which has a number of complications and complexities).

## Compiling


Compile with:

```
ant
```
Create a jar that you can use with other projects with:

```
ant dist
```

Alternatively, create a jar that includes the dependencies with 

```
ant dist_all
```

In both cases, the jar files will be stored in the `dist` folder.

Create java doc with:

```
ant doc
```

The produced Java doc will be in the `doc` folder.

profit.


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

		RosBridge bridge = RosBridge.createConnection(args[0]);
		bridge.waitForConnection();


		bridge.subscribe("/ros_to_java", "std_msgs/String",
				new RosListenDelegate() {
					@Override
					public void receive(JsonNode data, String stringRep) {
						MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(PrimitiveMsg.class);
						PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
						System.out.println(msg.data);
					}
				}, 1, 1);



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

As the `RosTest` code illustrates, the main Java object you work with is an instance of the `RosBridge` class that is defined in the `ros` package. `RosBridge` objects are created using the static method `createConnection(String rosBridgeURI)` (which takes the URI of the ROS Bridge server). Note that after creating a connection it is a good idea to call the `waitForConnection()` method, which will block the client thread until the connection with the ROS Bridge server has been established. 

A client Java object interacts with a `RosBridge` object with publish commands and subscription requests. Publish commands can be given with the `publish(String topic, String type, Object msg)` method, which automatically advertises the publish topic if it has not yet already been advertised. A more streamlined and ROS-like way to handle publishing (and what is demonstrated in the `RosTest` code) is to create a `Publisher` object instance (also defined in the `ros` package), which maintains the topic name and ROS topic message type so that you only need to pass its `publish(Object)` method the message you wish to publish. The message object you pass the publish method can either be a [Java Bean](https://en.wikipedia.org/wiki/JavaBeans) whose data members correspond to the ROS message structure, or a Java Map data structure whose keys are strings representing the names of the ROS message fields and values are the corresponding data type or another nested Map. (Note that a [Java Bean](https://en.wikipedia.org/wiki/JavaBeans) is a Java object with a default constructor and either public data members or data members that have setter and getter methods following standard Java method name conventions.) `java_rosbridge` includes some Java Bean data structures for common ROS messages in the `msgs` package. For example, the `PrimitiveMsg` is a Java Bean that can be used for many of the primitive ROS message types found in the `std_msgs` ROS package by setting its generic type to the appropriate primitive. For example, `PrimitiveMsg<String>` can be used for the `std_msgs/String` message. Alternatively, if you wanted to recreate the `std_msgs/String` message with a Java Map, the Map should contain one key-value entry of `data: stringValue`, where `stringValue` is whatever the ROS `std_msgs/String` data field value is.

You can subscribe to ROS topics via the `subscribe(String topic, String type, RosListenDelegate delegate)` or `subscribe(String topic, String type, RosListenDelegate delegate, int throttleRate, int queueLength)` method, which take a `RosListenDelegate` object (an interface defined in the `ros` package) which is a callback object that is passed the data sent from ROS Bridge for the specific topics to which you subscribed. You will need to implement your own `RosListenDelegate` object to process the received data, similar to how in normal Python ROS you implement a callback function to process subscriptions.
**Recommendation**: if you are subscribing to a high frequency topic, you should set the subcribed `throttleRate` and `queueLength` to 1 (or some other small values), otherwise your received data will increasingly lag behind the current real time values.

### Implementing a RosListenDelegate

A `RosListenDelegate` is an interface to define the callback function for subscribed topic messages sent over ROSBridge. It requires that you implement the method `receive(JsonNode data, String stringRep)`. The two parameters of this message present the ROSBridge message in two different formats. The latter, `stringRep`, is the string representation of the ROSBridge message which allows you to do anything you want with the raw data. The former, `data`, is a [JsonNode](http://fasterxml.github.io/jackson-databind/javadoc/2.2.0/com/fasterxml/jackson/databind/JsonNode.html) of the data sent over ROSBridge. `data` has four top-level JSON fields:

- `op`: which kind of messag operation it was; should always be "publish" (ROSBridge is passing a published message)
- `topic`: to which topic the message was published
- `type`: the ROS message type of the topic
- `msg`: the provided ros message in JSON format

The `msg` field is the primary field of interest since it contains the actual ROS message. The simplest way to parse a ROS message is to unpack it into a corresponding Java Bean for the ROS message type. The `MessageUnpacker` class can help you do that. As shown in the example code, a `MessageUnpacker` takes a generic specifying the type of Java Bean into which you're going to unpack the message and its constructor also requries you to give the Java .class reference. Then you can use its `unpackRosMessage(JsonNode)` method, providing it the `JsonNode` given to the `RosListenDelegate` `receive` method, as shown in the example code. (Note that you do not need to unpack the msg field first, the `MessageUnpacker` method will automatically do that for you.)

Alternatively, if you do not have a Java Bean class into which you can trivially unpack the ROS message, you can also iterate through the fields of the `JsonNode` manually. Working with JSON data using the `JsonNode` data structure is very easy because it has getter methods for JSON fields and elements in JSON arrays. For example, if the ROS message is a `geometry_msgs/Twist.msg` message, and you want the linear x component, you can retreive it with the code:

`double x = data.get("msg").get("linear").get("x").asDouble();`.

If a field's value is an array, it still is returned as a `JsonNode`; however, `JsonNode` has convenient methods for working with it. To get the size of the array use `node.size()` where `node` is the current `JsonNode` element you're using. To get an element within it (also returned as a `JsonNode` for recrusion of non-primitives), use `node.get(i)`, where `i` is the index into the array. See the [JsonNode Java documentation](http://fasterxml.github.io/jackson-databind/javadoc/2.2.0/com/fasterxml/jackson/databind/JsonNode.html) for more information on working with a `JsonNode`.

###### Interfacing with the Legacy Data Format

The earlier version of java_rosbridge (now saved in branch v1), has the `receive` method receive a `Map<String, Object>` data structure for the formatted JSON data, rather than a `JsonNode`. If your code was built on this legacy format and you would like to easily port things over, there is a `LegacyFormat` class within the `RosListenDelegate` interface that allows you to easily get back the `Map<String, Object>` representation of the JSON data. Simply use the code:

`Map<String, Object> oldFormat = RosListenDelegate.LegacyFormat.legacyFormat(stringRep);`

where `stringRep` is the string representation of the ROSBridge passed to the `receive` method, to get back this old format.
