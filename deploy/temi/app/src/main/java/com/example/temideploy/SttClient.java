package com.example.temideploy;

import android.util.Log;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;

public class SttClient extends Thread {

    private Socket sock;
    private DataInputStream input   = null;
    private DataOutputStream out     = null;
    private MainActivity main;

    public SttClient(MainActivity main) {
        this.main = main;
    }

    public void run() {

        // establish a connection
        try
        {
            sock = new Socket("192.168.0.139", 7779);
            System.out.println("Connected");

            // takes input from terminal
            input  = new DataInputStream(sock.getInputStream());

        }
        catch(UnknownHostException u)
        {
            Log.d("EXCEPTION", String.valueOf(u));
        }
        catch(IOException i)
        {
            Log.d("EXCEPTION", String.valueOf(i));
        }

        // string to read message from input
        String line = "";

        // keep reading until "Over" is input
        while (line == null || !line.equals("Over"))
        {
            try
            {
                line = input.readLine();
                if (line != null)
                    Log.d("STT",line);
                    line = line.replace("\n", "");
                    if (line.equals("begin interaction")) {
                        this.main.freezeTrigger(false);
                        this.main.resetInteraction();
                    }
                    else if (line.equals("terminate interaction")) {
                        Log.d("STT", "received termination request");
                        this.main.freezeTrigger(false);
                        this.main.endInteraction();
                    }
                    else if (line.equals("wake_word_active")) {
                        Log.d("STT", "received activate wake request");
                        this.main.freezeTrigger(true);
                        this.main.temiWakeWord(true);
                    }
                    else if (line.equals("wake_word_inactive")) {
                        Log.d("STT", "received deactivate wake request");
                        this.main.freezeTrigger(false);
                        this.main.temiWakeWord(false);
                    }
                    else if (line.equals("shutdown")) {
                        Log.d("STT", "received shutdown notice");
                        this.main.freezeTrigger(false);
                        this.main.shutdown();
                    }
                    else if (line.equals("debug")) {
                        Log.d("STT","received request to enter debug mode");
                        this.main.freezeTrigger(false);
                        this.main.toggleDebug(true);
                    }
                    //else if (line.equals("load interaction")) {
                    //   Log.d("STT", "received load request");
                    //    this.main.listenForInteraction();
                    //}
                    else {
                        Log.d("STT", "heard " + line);
                        this.main.freezeTrigger(false);
                        this.main.temiListen(line.toLowerCase());
                    }
            }
            catch(Exception i)
            {
                Log.d("EXCEPTION", String.valueOf(i));
                break;
            }
        }

        // close the connection
        try
        {
            input.close();
            out.close();
            sock.close();
        }
        catch(Exception i)
        {
            Log.d("EXCEPTION", String.valueOf(i));
        }
    }

}
