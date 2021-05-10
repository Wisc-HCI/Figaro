package com.example.temideploy;

import android.util.Log;

import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.Socket;
import java.net.UnknownHostException;

public class DeployClient {

    private Socket sock;
    private DataInputStream input   = null;
    private DataOutputStream out     = null;
    private MainActivity main;
    public final static int FILE_SIZE = 1800000;

    public DeployClient(MainActivity main) {
        this.main = main;
    }

    public void load() {

        // establish a connection
        try
        {
            sock = new Socket("192.168.0.139", 7780);
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

        // keep attempting to read an interaction file
        FileOutputStream fos = null;
        BufferedOutputStream bos;
        String FILE_TO_RECEIVED = "interaction_.xml";
        int bytesRead;
        int current = 0;
        try {

            // receive file
            byte [] mybytearray  = new byte [FILE_SIZE];
            InputStream is = sock.getInputStream();
            fos = new FileOutputStream(FILE_TO_RECEIVED);
            bos = new BufferedOutputStream(fos);
            bytesRead = is.read(mybytearray,0,mybytearray.length);
            current = bytesRead;

            do {
                bytesRead =
                        is.read(mybytearray, current, (mybytearray.length-current));
                if(bytesRead >= 0) current += bytesRead;
            } while(bytesRead > -1);

            bos.write(mybytearray, 0 , current);
            bos.flush();
            System.out.println("File " + FILE_TO_RECEIVED
                    + " downloaded (" + current + " bytes read)");
        } catch (IOException e) {
            e.printStackTrace();
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