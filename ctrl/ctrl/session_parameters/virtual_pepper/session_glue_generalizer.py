from synthesis.helper_superclasses.super_glue_generalizer import GlueGeneralizerSuper

class GlueGeneralizer(GlueGeneralizerSuper):

	def __init__(self):
		super().__init__()

	def generalize(self,glue,event_beh_classifier, main=False):

		return glue