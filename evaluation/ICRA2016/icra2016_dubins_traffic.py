from __future__ import print_function
import sys
import json
import rosbag
import numpy as np
import numpy.linalg as la


from fmrb import dubins_traffic

with open(sys.argv[1], 'rt') as fp:
    trialdata = json.load(fp)
rnd = dubins_traffic.RoadNetwork(trialdata['trialconf']['rnd'])

assert trialdata['trialconf']['number_goals_bounds'][0] == trialdata['trialconf']['number_goals_bounds'][1]
number_of_goals = trialdata['trialconf']['number_goals_bounds'][0]

assert trialdata['trialconf']['duration_bounds'][0] == trialdata['trialconf']['duration_bounds'][1]
duration = trialdata['trialconf']['duration_bounds'][0]

bag = rosbag.Bag(sys.argv[2])


ego_positions = []
ego_timestamps = []
agents = dict()

for topic, msg, timestamp in bag.read_messages(topics='/gazebo/model_states'):
    for idx, model_name in enumerate(msg.name):
        if model_name == 'ego':
            ego_positions.append([msg.pose[idx].position.x,
                                  msg.pose[idx].position.y])
            ego_timestamps.append(timestamp)
        elif model_name.startswith('agent'):
            if model_name not in agents:
                agents[model_name] = []
            agents[model_name].append([msg.pose[idx].position.x,
                                       msg.pose[idx].position.y])
    assert idx < len(msg.name)

for agentname in agents:
    assert len(ego_positions) == len(agents[agentname])


ego_positions = np.array(ego_positions)

if len(agents) > 0:
    min_dist = dict()
    for agentname in agents:
        agents[agentname] = np.array(agents[agentname])
        min_dist[agentname] = np.min(la.norm(ego_positions - agents[agentname], axis=1))

    if np.min(min_dist.values()) < 0.36:
        print(-1)  # Collision
        sys.exit()

counting = False
start_idx = None
duration_off_road = None
for idx, position in enumerate(ego_positions):
    if not rnd.on_road(position[0], position[1]) and not counting:
        counting = True
        start_idx = idx
    elif rnd.on_road(position[0], position[1]) and counting:
        counting = False
        if duration_off_road is None:
            duration_off_road = ego_timestamps[idx] - ego_timestamps[start_idx]
        else:
            duration_off_road += ego_timestamps[idx] - ego_timestamps[start_idx]

if duration_off_road is None:
    duration_off_road = 0
else:
    duration_off_road = duration_off_road.to_sec()

goal_visits = set()
cycle_counts = 0
for idx, position in enumerate(ego_positions):
    for goal_idx, goal in enumerate(trialdata['instances'][0]['goals']):
        mapped_goal = np.array(rnd.map_point(goal[0], goal[1]))
        if la.norm(position - mapped_goal) < trialdata['instances'][0]['intersection_radius']:
            goal_visits.add(goal_idx)

    if goal_visits == set(range(len(trialdata['instances'][0]['goals']))):
        cycle_counts += 1
        goal_visits = set()

print('cycle count: ', cycle_counts)

score = len(agents.keys())
score *= rnd.shape[0] * rnd.shape[1]
score *= cycle_counts
score /= max(1, duration_off_road)

print(score)
