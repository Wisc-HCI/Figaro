package com.example.temideploy;

import android.util.Log;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.w3c.dom.Document;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.io.StringReader;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import sensor_msgs.Image;
import std_msgs.String;

public class RosNode implements NodeMain {

    private MainActivity main;

    public RosNode(MainActivity main) {
        this.main = main;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final Subscriber<String> speechSub = connectedNode.newSubscriber("/temi/nlp",std_msgs.String._TYPE);
        final Subscriber<String> orientationSub = connectedNode.newSubscriber("/temi/orientation",std_msgs.String._TYPE);
        final Subscriber<String> deploySub = connectedNode.newSubscriber("/deploy/messages",std_msgs.String._TYPE);

        final Publisher<Image> imPub = connectedNode.newPublisher("/temi/camera",sensor_msgs.Image._TYPE);

        deploySub.addMessageListener(new MessageListener<String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {

                java.lang.String data = message.getData();
                java.lang.String topic = data.substring(0,data.indexOf("###"));
                data = data.substring(data.indexOf("###")+3,data.length());

                if (topic.equals("wipe_interrupts")) {
                    main.clearInterrupts();
                } else if (topic.equals("speech_xml")) {
                    java.lang.String content = data;
                    Log.d("SPEECH XML", content);
                    Document speech_document = getDocFromXML(content);
                    main.loadSpeechExamples(speech_document);
                    main.setStatusText("Loaded interaction");
                    main.setStatusText("loaded: speech");
                } else if (topic.equals("main")) {
                    java.lang.String content = data.substring(0,data.indexOf("###"));
                    Log.d("MAIN", content);
                    java.lang.String name = data.substring(data.indexOf("###")+3,data.length());
                    Document ts_document = getDocFromXML(content);
                    main.loadInteraction(ts_document);
                } else if (topic.equals("interrupt")) {
                    java.lang.String content = data.substring(0,data.indexOf("###"));
                    Log.d("INTERRUPT", content);
                    java.lang.String name = data.substring(data.indexOf("###")+3,data.length());
                    Document ts_document = getDocFromXML(content);
                    main.loadInterrupt(ts_document);
                }
            }
        });

        speechSub.addMessageListener(new MessageListener<String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                java.lang.String data = message.getData();
                java.lang.String topic = data.substring(0,data.indexOf("###"));
                data = data.substring(data.indexOf("###")+3,data.length());

                if (topic.equals("heyrobot")) {
                    if (data.equals("wake_word_active")) {
                        main.freezeTrigger(true);
                        main.temiWakeWord(true);
                    } else if (data.equals("wake_word_inactive")) {
                        main.freezeTrigger(false);
                        main.temiWakeWord(false);
                    } else if (data.equals("shutdown")) {
                        main.freezeTrigger(false);
                        main.shutdown();
                    } else if (data.equals("debug")) {
                        Log.d("STT", "received request to enter debug mode");
                        main.freezeTrigger(false);
                        main.toggleDebug(true);
                    } else {
                        Log.d("STT", "heard " + data);
                        main.freezeTrigger(false);
                        main.temiListen(data.toLowerCase());
                    }
                }
            }
        });

        orientationSub.addMessageListener(new MessageListener<String>() {
            @Override
            public void onNewMessage(std_msgs.String message) {
                java.lang.String data = message.getData();
                java.lang.String angleString = data.substring(0,data.indexOf("###"));
                java.lang.String timeString = data.substring(data.indexOf("###")+3,data.length());

                double angle = Double.parseDouble(angleString);
                double timestamp = Double.parseDouble(timeString);
                main.updateCurrOrientation(angle,timestamp);
            }
        });

    }
    @Override
    public void onShutdown(Node node) {
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("Temi");
    }

    public Document getDocFromXML(java.lang.String xml) {
        Document document = null;
        // read in new data
        DocumentBuilderFactory docBuilderFactory = DocumentBuilderFactory.newInstance();
        DocumentBuilder docBuilder = null;
        try {
            docBuilder = docBuilderFactory.newDocumentBuilder();
        } catch (ParserConfigurationException e) {
            e.printStackTrace();
        }
        try {
            document = docBuilder.parse(new InputSource(new StringReader(xml)));

        } catch (IOException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        }

        return document;
    }

    public void publishCameraImage() {

    }
}