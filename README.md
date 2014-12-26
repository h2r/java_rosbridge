java_rosbridge
==============

Simple library using Jetty 9 to connect Java to a rosbridge server. Supports publishing and subscribing with different topic delegates


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

In both cases, the jar files will be stored in the dist folder.

Create java doc with:

```
ant doc
```

Produced Java doc will be in the doc folder.

profit.

==============
Here we will provide a brief description of the library code, but you should consult the Java doc for more detailed information.

First thing to note is the tests package shows some very basic test code that you can run. The SimpleEchoClient class runs a very simple example of websockets running using Jetty; it does not have anything specifically to do with ROS and is a way of sanity checking the websocket code. It makes reference to the SimpleEchoSocket class for handling the communication. All this code was provided by the Jetty tutorial. The echo.websocket.org server to which the code connects is a simple websocket server for testing websockets (also specified for use in the Jetty tutorial). In addition to a connect printout, you should see the following printed to the terminal:

```
Got msg: Hello
Got msg: Thanks for the conversation.
Connection closed: 1000 - I'm done
```


The RosTest code provides a demonstration of connecting to ROSBridge and publishing the string "hello from java" to the ROS topic /bridge every 500 ms. Note that ROSBridge by default runs on port 9090. An example URI is: ws://localhost:9090. Obviously, you should launch rosbridge on the server first. To observe the publish on the ROS server, run the ROS command

```
rostopic echo /bridge
```

The RosTest code also has some commented code that demonstrates how you subscribe to topics over ROSbridge. The commented out code assumes the ROS message type: "burlap_msgs/burlap_state" change that as needed. It will simply print the JSON string it receives every time it receives an update from the server.


As the RosTest code illustrates the main object you work with is the RosBridge class in the ros package. This package receives publish and subscribe requests. Publish requests automatically advertise the publish if it has not yet already been advertised. Subscribe requests take a RosListenDelegate object (interface defined in the ros package) which is an object which is passed the data sent from ROS bridge for specific topics to which you subscribed.

