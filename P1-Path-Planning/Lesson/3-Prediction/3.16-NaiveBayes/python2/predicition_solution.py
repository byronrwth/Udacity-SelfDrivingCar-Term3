import numpy as np
import random
from math import sqrt, pi, exp
import json

def gaussian_prob(obs, mu, sig):
    num = (obs - mu)**2
    denum = 2 * sig**2
    norm = 1 / sqrt(2 * pi * sig**2)
    return norm * exp(-num / denum)


class GNB():

    def __init__(self):
        self.classes = ['left', 'keep', 'right']

    def process_vars(self, vars):
        # could do something fancy in here, but right now
        # s, d, s_dot and d_dot alone give good results
        s, d, s_dot, d_dot = vars
        return s, d, s_dot, d_dot

    def train(self, X, Y):
        """
        X is an array of training data, each entry of which is a 
        length 4 array which represents a snapshot of a vehicle's
        s, d, s_dot, and d_dot coordinates.

        Y is an array of labels, each of which is either 'left', 'keep',
        or 'right'. These labels indicate what maneuver the vehicle was 
        engaged in during the corresponding training data snapshot. 
        """

        num_vars = 4

        # initialize an empty array of arrays. For this problem
        # we are looking at three labels and keeping track of 4
        # variables for each (s,d,s_dot,d_dot), so the empty array
        # totals_by_label will look like this:

        # {
        #   "left" :[ [],[],[],[] ],
        #   "keep" :[ [],[],[],[] ],
        #   "right":[ [],[],[],[] ]
        # }

        totals_by_label = {
            "left": [],
            "keep": [],
            "right": [],
        }
        for label in self.classes:
            for i in range(num_vars):
                totals_by_label[label].append([])
                #print "totals_by_label["+label+"]= ", totals_by_label[label]

        for x, label in zip(X, Y):

            # process the raw s,d,s_dot,d_dot snapshot if desired.
            x = self.process_vars(x)

            # add this data into the appropriate place in the
            # totals_by_label data structure.
            for i, val in enumerate(x):
                #print "i= ",i, " val= ",val
                """
                i=  0  val=  4.76476266072
                i=  1  val=  0.283424594434
                i=  2  val=  9.93421512774
                i=  3  val=  0.111951162108
                i=  0  val=  39.7084757287
                i=  1  val=  7.60549751981
                i=  2  val=  8.15974055685
                i=  3  val=  0.401671056683
                i=  0  val=  29.8549416734
                i=  1  val=  1.20617431037
                i=  2  val=  11.0470575581
                i=  3  val=  -1.47480653235

                """ 
                totals_by_label[label][i].append(val)
                #print "totals_by_label["+label+"]= ", totals_by_label[label]
                """
                totals_by_label[keep]=  [[35.548519767216185, 31.336231964389462, 22.15610950248377, 14.767391310925861, 6.449170258268785, 14.052456024041158, ...]]
            
                """
        #print "totals_by_label[right][0]= ", totals_by_label["right"][0]  -->s, 
        #print "totals_by_label[right][1]= ", totals_by_label["right"][1]  -->d
        ##print "totals_by_label[right][2]= ", totals_by_label["right"][1]  -->s_dot
        #print "totals_by_label[right][3]= ", totals_by_label["right"][1]  --> d_dot

        # Get the mean and standard deviation for each of the arrays
        # we've built up. These will be used as our priors in GNB
        means = []
        stds = []
        for i in self.classes: # right, keep, left
            means.append([])
            stds.append([])
            for arr in totals_by_label[i]:
                mean = np.mean(arr)
                std = np.std(arr)
                means[-1].append(mean)
                stds[-1].append(std)

        self._means = means
        self._stds = stds

    def _predict(self, obs):
        """
        Private method used to assign a probability to each class.
        """
        probs = []
        obs = self.process_vars(obs)
        for (means, stds, lab) in zip(self._means, self._stds, self.classes):
            product = 1
            for mu, sig, o in zip(means, stds, obs):
                likelihood = gaussian_prob(o, mu, sig)
                product *= likelihood
            probs.append(product)
        t = sum(probs)
        return [p / t for p in probs]

    def predict(self, observation):
        probs = self._predict(observation)
        idx = 0
        best_p = 0
        for i, p in enumerate(probs):
            if p > best_p:
                best_p = p
                idx = i
        names = ['left', 'keep', 'right']
        return names[idx]


def main():
    gnb = GNB()
    with open('train.json', 'rb') as f:
        j = json.load(f)

    # print j.keys()
    X = j['states']
    Y = j['labels']

    # print len(X), len(Y)
    print "-----start training-------"
    gnb.train(X, Y)

    with open('test.json', 'rb') as f:
        j = json.load(f)

    X = j['states']
    Y = j['labels']
    score = 0
    for coords, label in zip(X, Y):
        predicted = gnb.predict(coords)
        if predicted == label:
            score += 1
    fraction_correct = float(score) / len(X)
    print "You got {} percent correct".format(100 * fraction_correct)

if __name__ == "__main__":
    main()

#-----start training-------
#You got 84.4 percent correct