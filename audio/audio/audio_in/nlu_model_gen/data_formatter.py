import json
import sys
import pickle
import numpy as np

class Generator():

    def __init__(self):
        self.file = None

    def open_training_file(self):
        try:
            self.file = open("pos_examples.txt","r")
        except:
            print("ERROR: could not open text file with raw training examples")
            exit(1)

    def close_training_file(self):
        try:
            self.file.close()
        except:
            print("ERROR: could not close text file with raw training examples")
            exit(1)

    def generate(self):
        self.open_training_file()
        training_examples = []
        for line in self.file:

            training_utterance_split = line.strip().split()
            training_example = []

            curr_in = "NONE"
            for word in training_utterance_split:
                if word == "<":
                    #curr_in = "ROBOTSPEECH"
                    curr_in = "CONTENT"
                    continue
                elif word == ">":
                    curr_in = "NONE"
                    continue
                elif word == "<<":
                    curr_in = "ROBOTACTION"
                    continue
                elif word == ">>":
                    curr_in = "NONE"
                    continue
                elif word == "<<<":
                    curr_in = "HUMANSPEECH"
                    continue
                elif word == ">>>":
                    curr_in = "NONE"
                    continue
                elif word == "<<<<":
                    curr_in = "HUMANACTION"
                    continue
                elif word == ">>>>":
                    curr_in = "NONE"
                    continue

                training_example.append((word,curr_in))

            training_examples.append(training_example)

        with open("generated_data.pkl","wb") as outfile:
            pickle.dump(training_examples,outfile)

        self.close_training_file()

if __name__=='__main__':
    '''

    '''
    generator = Generator()
    generator.generate()
