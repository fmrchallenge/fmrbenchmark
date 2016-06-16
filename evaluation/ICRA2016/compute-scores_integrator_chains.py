from __future__ import print_function
import sys
import csv

import icra2016_integrator_chains


scores = []
with open(sys.argv[1], 'rt') as fp:
    csv_reader = csv.reader(fp)
    # each row is the name of a trial data file
    for row in csv_reader:
        print('Scoring trial ', len(scores), '...')
        scores.append(icra2016_integrator_chains.get_integrator_chains_score(row[0]))
print('scores: ', scores)
print('sum: ', reduce(lambda x,y: x+y, scores))
