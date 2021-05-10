from rasa.nlu.training_data import load_data
from rasa.nlu.config import RasaNLUModelConfig
from rasa.nlu.model import Trainer, Metadata, Interpreter
from rasa.nlu import config

def train_model(data_file):

	print('Training new model. This will take a minute')
	training_data = load_data(data_file)
	trainer = Trainer(config.load("config/config_spacy.yml"))
	trainer.train(training_data)
	model_directory = trainer.persist('.')
	return model_directory

if __name__=="__main__":
	'''
	Train the following models:
		- robot speech
		- robot actions
		- human speech
		- human actions
	'''
	train_model("speech_intents.json")
	train_model("action_intents.json")