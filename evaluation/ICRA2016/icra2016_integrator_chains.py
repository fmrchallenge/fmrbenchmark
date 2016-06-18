from __future__ import print_function
import sys
import json
import rosbag
import numpy as np
import numpy.linalg as la

from fmrb import integrator_chains
import tdstat



def check_sat(td, trial_index):
# checks satisfaction of reach-avoid specifications, i.e. 
# of the form /\_i G(not obs_i) /\ /\_j F(goal_j)
    try:
        traj = np.array(td['trials'][trial_index]['trajectory'])
    except KeyError:
        print('Trial '+str(trial_index)+' does not have an associated trajectory.')
        sys.exit(-1)
    prob = integrator_chains.Problem.loadJSONdict(td['trials'][trial_index]['problem_instance'])
    goals_satisfied = [0]*len(prob.goals)
    for row in traj:
        y = row[2:(2+prob.output_dim)]
        y_label = []
        for lpolytope in prob.obstacles:
            if lpolytope.contains(y):
                return False
        for i in range(len(prob.goals)):
            lpolytope = prob.goals[i]
            if lpolytope.contains(y):
                goals_satisfied[i] = 1
                
    if all(goals_satisfied):
        return 1
    else:
        return -1
                


def get_integrator_chains_score(trialfile, bagfile=None):
    # TODO: use bag file instead of trial data
    score = 0

    with open(trialfile, 'rt') as fp:
        trialdata = json.load(fp)
    assert trialdata['version'] == 0

    for k in range(len(trialdata['trials'])):
        prob = integrator_chains.Problem.loadJSONdict(trialdata['trials'][k]['problem_instance'])

        try:
            realizable = trialdata['trials'][k]['realizable']
        except KeyError:
            realizable = None
            
        if realizable is None:
           # trial is skipped
           continue;         

        is_sat = check_sat(trialdata, k)
        
        nominal_duration = trialdata['trials'][k]['duration']

        start_time = trialdata['trials'][k]['start_time'][0]+trialdata['trials'][k]['start_time'][1]*1e-9
        
        try:
            decision_time = trialdata['trials'][k]['decision_time'][0]+trialdata['trials'][k]['decision_time'][1]*1e-9
            decision_diff = decision_time - start_time
        
        except KeyError:
            # no decision duration available, don't score this round
            continue            

        if trialdata['trials'][k].has_key('trajectory'):
            second_state_time = trialdata['trials'][k]['trajectory'][0][0]+trialdata['trials'][k]['trajectory'][0][1]*1e-9
            end_state_time = trialdata['trials'][k]['trajectory'][-1][0]+trialdata['trials'][k]['trajectory'][-1][1]*1e-9
            trajectory_duration = end_state_time - second_state_time
            
        try:
            traj_discrete_length = len(trialdata['trials'][k]['trajectory'])
        except KeyError:
            traj_discrete_length = None


        score += is_sat
        score /= decision_diff
        score *= prob.output_dim
        score *= prob.number_integrators

        ### for future versions
        # score *= prob.number_goals
        # score *= prob.number_obstacles
        
        return score
    
            
    


if __name__ == '__main__':
    print(get_chains_integrators_score(sys.argv[1]))
