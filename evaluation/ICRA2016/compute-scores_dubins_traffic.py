from __future__ import print_function
import sys
import csv

import icra2016_dubins_traffic


scores = []
with open(sys.argv[1], 'rt') as fp:
    csv_reader = csv.reader(fp)
    for row in csv_reader:
        print('Scoring trial ', len(scores), '...')
        scores.append(icra2016_dubins_traffic.get_dubins_traffic_score(row[0], row[1]))
print('scores: ', scores)
print('sum: ', reduce(lambda x,y: x+y, scores))
