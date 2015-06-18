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

The `RosTest` code in the `tests` package provides a demonstration of connecting to ROS Bridge and publishing the string `hello from java` to the ROS topic `/bridge` every 500 ms. The code takes as a command line argument the URI to the ROS Bridge server. Note that ROS Bridge by default runs on port 9090. An example URI is `ws://localhost:9090`. Obviously, you should launch ROS Bridge on the server before running this example code. (That is, before running the `RosTest` class, run `roslaunch rosbridge_server rosbridge_websocket.launch` on the ROS server. For more information on getting and starting ROS Bridge on your ROS server, see [here](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge).) To observe the Java publishing over ROS Bridge on the actual ROS Bridge server, run the ROS command

```
rostopic echo /bridge
```
on the ROS server. You should find it outputting the published message `hello from java`

The `RosTest` class code also has some commented out code that demonstrates how you can use the `java_rosbridge` library to subscribe to topics over ROS Bridge. The commented out code assumes the subscribed topic `/burlap_chatter` is broadcasting the ROS message type `burlap_msgs/burlap_state`. Change the topic and message type to whatever existing ROS topic on which you want to test a subscription. The result of the commented out code will print the JSON string it receives every time it receives an update from the server. Note that the `java_rosbridge` library also parses that JSON string into a convenient `Map<String,Object>` data structure for so that you don't have to parse the JSON string yourself (see the Java doc for more information), but for the demo code, it's only printing out the unparsed string.


### Primary Library Classes

As the `RosTest` code illustrates, the main Java object you work with is an instance of the `RosBridge` class that is defined in the `ros` package. `RosBridge` objects are created using the static method `createConnection(String rosBridgeURI)` (which takes the URI of the ROS Bridge server). Note that after creating a connection it is a good idea to call the `waitForConnection()` method, which will block the client thread until the connection with the ROS Bridge server has been established. 

A client Java object interacts with a `RosBridge` object with publish commands and subscription requests. Publish commands can be given with the `publish(String topic, String type, Object msg)` method, which automatically advertises the publish topic if it has not yet already been advertised. A more streamlined and ROS-like way to handle publishing (and what is demonstrated in the `RosTest` code) is to create a `Publisher` object instance (also defined in the `ros` package), which maintains the topic name and ROS topic message type so that you only need to pass its `publish(Object)` method the message you wish to publish. The message object you pass the publish method in general should be a `Map` object of some sort that specifies the data of the ROS message type. For example, if the ROS topic message type is `std_msgs/String` then the message object should be a `Map<String,String>` object with one Map entry of `data: stringValue`, where `stringValue` is whatever the ROS `std_msgs/String` data field value is.

Subscribe requests via the `subscribe(String topic, String type, RosListenDelegate delegate)` or `subscribe(String topic, String type, RosListenDelegate delegate, int throttleRate, int queueLength)` method, which take a `RosListenDelegate` object (an interface defined in the `ros` package) which is a callback object that is passed the data sent from ROS Bridge for the specific topics to which you subscribed. You will need to implement your own `RosListenDelegate` object to process the received data, similar to how in normal Python ROS you implement a callback function to process subscriptions.
**Recommendation**: if you are subscribing to a high frequency topic, you should set the subcribed `throttleRate` and `queueLength` to 1 (or some other small values), otherwise your received data will increasingly lag behind the current real time values.

