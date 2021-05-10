package com.example.temideploy;

import android.content.Context;
import android.util.Log;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

public class SpeechDatabase {

    HashMap<String, ArrayList<String>> db;

    public SpeechDatabase() {
        this.db = new HashMap<String, ArrayList<String>>();
    }

    public void readSpeechXml(Context context, String filename) {
        Log.d("LOADING","Filename is " + filename);

        try {
            InputStream is = context.getAssets().open(filename);
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder;
            dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(is);
            this.readSpeechDocument(doc);

        } catch (SAXException | ParserConfigurationException | IOException e1) {
            e1.printStackTrace();
        }
    }

    public void readSpeechDocument(Document doc) {
        doc.getDocumentElement().normalize();
        System.out.println("Root element :" + doc.getDocumentElement().getNodeName());

        // reinitialize the database
        this.db = new HashMap<String, ArrayList<String>>();

        NodeList catlist = doc.getElementsByTagName("speech_cat");

        for (int i = 0; i < catlist.getLength(); i++) {

            Node nNode = catlist.item(i);
            Element elem = (Element) nNode;
            String cat = elem.getAttribute("id");

            NodeList phraseList = elem.getElementsByTagName("phrase");
            ArrayList<String> phrases = new ArrayList<String>();
            for (int j = 0; j < phraseList.getLength(); j++) {
                Node phrase = phraseList.item(j);
                Element phraseElem = (Element) phrase;
                String phraseString = phraseElem.getAttribute("text");
                phrases.add(phraseString.toLowerCase());
            }

            this.db.put(cat.toLowerCase(),phrases);

        }
    }

    public boolean inSpeechDb(String text) {
        if (this.db.containsKey(text))
            return true;
        return false;
    }

    public String getRandomPhrase(String cat) {
        ArrayList<String> phraseList = this.db.get(cat);
        if (phraseList.size() == 0)
            return cat;
        Random rand = new Random();
        return phraseList.get(rand.nextInt(phraseList.size()));
    }

    public String toString() {
        return String.valueOf(this.db);
    }

}
