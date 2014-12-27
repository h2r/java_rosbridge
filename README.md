java_rosbridge
==============

Simple library using Jetty 9 to connect Java to a ROS Bridge server. Supports publishing and subscribing with different topic delegates.

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

---

## Using the code

Here we will provide a brief description of the library code, but you should consult the Java doc for more detailed information.

### Websockets test code

The first thing to note is the `tests` package shows some very basic test code that you can run. The `SimpleEchoClient` class runs a very simple example of websockets running using Jetty; it does not have anything specifically to do with ROS and is a way of sanity checking the websocket code. It makes reference to the `SimpleEchoSocket` class for handling the communication. All this code was provided by the Jetty tutorial. The `echo.websocket.org` server to which the code connects is a simple websocket server for testing websockets (also specified for use in the Jetty tutorial). When you run it, you should get a long connect debug message printed to the terminal and the following which are the messages received from the `echo.websocket.org` server:

```
Got msg: Hello
Got msg: Thanks for the conversation.
Connection closed: 1000 - I'm done
```


### ROS Bridge test code

The RosTest code in the `tests` package provides a demonstration of connecting to ROSBridge and publishing the string `hello from java` to the ROS topic /bridge every 500 ms. The code takes as a command line argument that is the URI to the ROS Bridge server. Note that ROSBridge by default runs on port 9090. An example URI is: ws://localhost:9090. Obviously, you should launch ROS Bridge on the server before running this example code. To observe the Java publishing over ROS Bridge on the actual ROS Bridge server, run the ROS command

```
rostopic echo /bridge
```
on the ROS server. You should find it outputting the published message `hello from java`

The RosTest code also has some commented code that demonstrates how you can use the library to subscribe to topics over ROSbridge. The commented out code assumes the subscribed topic `/burlap_chatter' is broadcasting the ROS message type: `burlap_msgs/burlap_state`. Change the topic and message type to whatever existing ROS topic on which you want to test a subscription. The result of the commented out code will print the JSON string it receives every time it receives an update from the server. Note that the library we've created also parses that JSON string into a convenient `Map<String,Object>` data structure for use as well so that you don't have to parse the JSON string yourself (see the Java doc for more information).


### Primary Library Classes

As the RosTest code illustrates the main object you work with is the `RosBridge` class in the `ros` package. `RosTest` objects are created using the static method `createConnection(String)` (which takes the URI of the ROSBridge server). A client object interacts with a `RosTest` object with publish commands and subscription requests. Publish requests automatically advertise the publish topic if it has not yet already been advertised. Subscribe requests take a `RosListenDelegate` object (an interface defined in the `ros` package) which is an object that is passed the data sent from ROS bridge for specific topics to which you subscribed. You will need to implement your own `RosListenDelegate` object to process the received data, similar to how in normal Python ROS you implement a callback function to process subscriptions.

