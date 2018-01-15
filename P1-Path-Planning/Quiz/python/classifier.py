

class GNB(object):

	def __init__(self):
		self.possible_labels = ['left', 'keep', 'right']

	def train(self, data, labels):
		"""
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
		"""
		item_nr_per_label = 4

		totals_by_label = {
		    "left" : [], 
		    "keep" : [],
		    "right": [],
		}

		for each_data, each_label in zip(data, labels):
			print "each data=", each_data
			print "each label=", each_label
			totals_by_label[each_label].append(each_data)

		for label in self.possible_labels:
			print label 
			print len(totals_by_label[label])

		for m in range(10):
			print (totals_by_label["left"])
			print (totals_by_label["keep"])
			print (totals_by_label["right"])

		#for label in self.possible_labels:
		#    for i in range(item_nr_per_label):
		#        totals_by_label[label].append([])
		#        print label
		#        print totals_by_label[label] 





		#for i in data:
	    #	print i
		#	print "\n"
		
		#for label in labels:
		#	print j

		#print 

		

	def predict(self, observation):
		"""
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
		return self.possible_labels[1]