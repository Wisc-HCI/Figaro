package com.example.temideploy;

import android.content.Intent;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import com.google.android.gms.tasks.OnCompleteListener;
import com.google.android.gms.tasks.Task;
import com.google.firebase.messaging.FirebaseMessaging;
import com.google.firebase.messaging.FirebaseMessagingService;
import com.google.firebase.messaging.RemoteMessage;

import java.util.Map;

public class FirebaseServer extends FirebaseMessagingService {
    private LocalBroadcastManager broadcaster;

    private String ts;

    public FirebaseServer() {
        // for data streams
        this.ts = null;
        this.initializeTopic();
    }

    @Override
    public void onCreate() {
        broadcaster = LocalBroadcastManager.getInstance(this);
    }

    @Override
    public void onMessageReceived(RemoteMessage remoteMessage) {
        // ...

        // TODO(developer): Handle FCM messages here.
        // Not getting messages here? See why this may be: https://goo.gl/39bRNJ
        Log.d("FB", "From: " + remoteMessage.getFrom());
        String topic = remoteMessage.getFrom();

        // Check if message contains a data payload.
        if (remoteMessage.getData().size() > 0) {
            Log.d("FB", "Message data payload: " + remoteMessage.getData());

            Intent intent = new Intent("MyData");

            //JsonObject data = new JsonObject(); // com.google.gson.JsonObject
            //JsonParser jsonParser = new JsonParser(); // com.google.gson.JsonParser
            Map<String, String> data = remoteMessage.getData();
            //String val;

            //for (String key : map.keySet()) {
            //    val = map.get(key);
            //    try {
            //        data.add(key, jsonParser.parse(val));
            //    } catch (Exception e) {
            //        data.addProperty(key, val);
            //    }
            //}

            if (topic.equals("/topics/heyrobot")) {
                String msgType = data.get("msgType");

                intent.putExtra("method", msgType);
                intent.putExtra("topic", "heyrobot");
                broadcaster.sendBroadcast(intent);
            }
            else if (topic.equals("/topics/open_xml")) {
                this.ts = "";
            }
            else if (topic.equals("/topics/xml_content")) {
                String tsContent = data.get("content");
                this.ts += tsContent;
            }
            else if (topic.equals("/topics/close_xml")) {
                String name = data.get("name");
                String isInterrupt = data.get("is_interrupt");
                this.ts = "<"+name+">"+this.ts+"</"+name+">";
                intent.putExtra("content", this.ts);

                if (isInterrupt.equals("interrupt")) {
                    intent.putExtra("topic", "interrupt");
                    broadcaster.sendBroadcast(intent);
                }
                else {
                    intent.putExtra("topic", "main_interaction");
                    broadcaster.sendBroadcast(intent);
                }
            }
            else if (topic.equals("/topics/speech_xml")) {
                String phrasesContent = data.get("phrases");
                intent.putExtra("content", phrasesContent);
                intent.putExtra("topic", "speech_xml");
                broadcaster.sendBroadcast(intent);
            }
            else if (topic.equals("/topics/wipe_interrupts")) {
                intent.putExtra("method", "clearInterrupts");
                intent.putExtra("topic", "wipe_interrupts");
                broadcaster.sendBroadcast(intent);
            }
            else if (topic.equals("/topics/orientation")) {
                String id = data.get("id");
                String angle = data.get("angle");
                Log.d("ANGLE", String.valueOf(angle));
                String timestamp = data.get("timestamp");
                intent.putExtra("angle", angle);
                intent.putExtra("timestamp", timestamp);
                intent.putExtra("topic", "orientation");
                broadcaster.sendBroadcast(intent);
            }
        }
    }

    public void initializeTopic() {

        FirebaseMessaging.getInstance().subscribeToTopic("heyrobot").addOnCompleteListener(new OnCompleteListener<Void>() {
            @Override
            public void onComplete(@NonNull Task<Void> task) {
                String msg = "successful";
                if (!task.isSuccessful()) {
                    msg = "unsuccessful";
                }
                Log.d("FB", "subscribing hey robot topic: " + msg);
                //Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });
        FirebaseMessaging.getInstance().subscribeToTopic("open_xml").addOnCompleteListener(new OnCompleteListener<Void>() {
            @Override
            public void onComplete(@NonNull Task<Void> task) {
                String msg = "successful";
                if (!task.isSuccessful()) {
                    msg = "unsuccessful";
                }
                Log.d("FB", "subscribing open_xml topic: " + msg);
                //Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });

        FirebaseMessaging.getInstance().subscribeToTopic("xml_content").addOnCompleteListener(new OnCompleteListener<Void>() {
            @Override
            public void onComplete(@NonNull Task<Void> task) {
                String msg = "successful";
                if (!task.isSuccessful()) {
                    msg = "unsuccessful";
                }
                Log.d("FB", "subscribing xml_content topic: " + msg);
                //Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });

        FirebaseMessaging.getInstance().subscribeToTopic("close_xml").addOnCompleteListener(new OnCompleteListener<Void>() {
            @Override
            public void onComplete(@NonNull Task<Void> task) {
                String msg = "successful";
                if (!task.isSuccessful()) {
                    msg = "unsuccessful";
                }
                Log.d("FB", "subscribing close_xml topic: " + msg);
                //Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });

        FirebaseMessaging.getInstance().subscribeToTopic("speech_xml").addOnCompleteListener(new OnCompleteListener<Void>() {
            @Override
            public void onComplete(@NonNull Task<Void> task) {
                String msg = "successful";
                if (!task.isSuccessful()) {
                    msg = "unsuccessful";
                }
                Log.d("FB", "subscribing speech_xml topic: " + msg);
                //Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });

        FirebaseMessaging.getInstance().subscribeToTopic("wipe_interrupts").addOnCompleteListener(new OnCompleteListener<Void>() {
            @Override
            public void onComplete(@NonNull Task<Void> task) {
                String msg = "successful";
                if (!task.isSuccessful()) {
                    msg = "unsuccessful";
                }
                Log.d("FB", "subscribing speech_xml topic: " + msg);
                //Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });

        FirebaseMessaging.getInstance().subscribeToTopic("orientation").addOnCompleteListener(new OnCompleteListener<Void>() {
            @Override
            public void onComplete(@NonNull Task<Void> task) {
                String msg = "successful";
                if (!task.isSuccessful()) {
                    msg = "unsuccessful";
                }
                Log.d("FB", "subscribing orientation topic: " + msg);
                //Toast.makeText(MainActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });

        // [END subscribe_topics]

    }

}
