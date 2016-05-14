import math
import json

# inline:
#   matplotlib


class RoadNetwork(object):
    def __init__(self, rnd_file, is_json=True):
        """Instantiate road network description

        rnd_file can be string (file name) or file-like object, or
        dict. In the former case, attempt to read it as RND file in
        the plaintext or JSON container (depending on is_json
        parameter). In the latter case (i.e., dict object), treat it
        as a file in the JSON container that is already loaded.

        Note that two containers are recognized: JSON and plaintext.
        If the argument is_json=True (default) the file is parsed as
        JSON. Otherwise, the plaintext format is used.
        """
        if (getattr(rnd_file, 'readline', None) is None
            and isinstance(rnd_file, (str, unicode))):
            rnd_file = open(rnd_file, 'rt')

        self.version = None
        self.length = None
        self.segments = list()
        self.transform = (0, 0, 0)
        self.shape = None

        if is_json:
            if isinstance(rnd_file, dict):
                rndjson = rnd_file
            else:
                rndjson = json.load(rnd_file)
            self.version = rndjson['version']
            assert self.version == 0, 'Unrecognized format version'
            self.length = rndjson['length']
            self.transform = tuple([float(x) for x in rndjson['transform']])
            if 'segments' in rndjson:
                self.segments = rndjson['segments']
            if 'shape' in rndjson:
                self.shape = tuple(rndjson['shape'])

        else:
            reading_segments = False
            for line in rnd_file:
                line = line.strip()
                if len(line) == 0 or line.startswith('#'):
                    continue  # Ignore comments, blank lines
                comment_index = line.find('#')
                if comment_index > 0:
                    line = line[:comment_index]  # Strip trailing comments
                if not reading_segments:
                    if line.startswith('version'):
                        self.version = int(line.split(':')[1])
                        assert self.version == 0, 'Unrecognized format version'
                    elif line.startswith('length'):
                        self.length = float(line.split(':')[1])
                    elif line.startswith('transform'):
                        self.transform = tuple([float(x) for x in line.split(':')[1].split()])
                    elif line.startswith('segments'):
                        reading_segments = True
                    elif line.startswith('shape'):
                        self.shape = tuple([float(x) for x in line.split(':')[1].split()])
                    else:
                        raise ValueError('Unrecognized file entry.')
                else:
                    self.segments.append([int(x) for x in line.split()])

        assert self.length > 0, 'Unit side length must be positive'
        assert len(self.transform) == 3, 'Map transform must be a 3-tuple'
        if self.shape is not None:
            assert len(self.shape) == 2, 'Shape must be a 2-tuple'
            assert self.shape[0] >= 1 and self.shape[1] >= 1, 'Shape must have positive elements'
        if len(self.segments) > 0:
            segments_shape = [0, 4]
            # extrema elements: x_min, x_max, y_min, y_max
            extrema = [self.segments[0][0], self.segments[0][1],
                       self.segments[0][0], self.segments[0][1]]
            for segment in self.segments:
                assert len(segment) == segments_shape[1], 'Mismatch in row length of segments'
                segments_shape[0] += 1
                extrema[0] = min(segment[0], segment[2], extrema[0])
                extrema[1] = max(segment[0], segment[2], extrema[1])
                extrema[2] = min(segment[1], segment[3], extrema[2])
                extrema[3] = max(segment[1], segment[3], extrema[3])
            if self.shape is None:
                self.shape = (extrema[3] - extrema[2],
                              extrema[1] - extrema[0])
            else:
                assert self.shape == (extrema[3] - extrema[2]+1,
                                      extrema[1] - extrema[0]+1), 'Shape and segments fields are not consistent'
        elif self.shape is not None:
            self.segments = []
            for x in range(self.shape[1]-1):
                for y in range(self.shape[0]-1):
                    self.segments.append([x, y, x+1, y])
                    self.segments.append([x, y, x, y+1])
            for x in range(self.shape[1]-1):
                self.segments.append([x, self.shape[0]-1,
                                      x+1, self.shape[0]-1])
            for y in range(self.shape[0]-1):
                self.segments.append([self.shape[1]-1, y,
                                      self.shape[1]-1, y+1])
        else:
            raise ValueError('At least one of segments or shape must be defined')

    def number_of_segments(self):
        return len(self.segments)

    def map_point(self, x, y):
        """Map point in local coordinates through transform and scaling.
        """
        costheta = math.cos(self.transform[2])
        sintheta = math.sin(self.transform[2])
        mapped_x = self.length*x
        mapped_y = self.length*y
        new_x = costheta*mapped_x - sintheta*mapped_y
        new_y = sintheta*mapped_x + costheta*mapped_y
        mapped_x = new_x + self.transform[0]
        mapped_y = new_y + self.transform[1]
        return (mapped_x, mapped_y)

    def get_mapped_segment(self, index):
        if index < 0 or index > len(self.segments)-1:
            raise ValueError('index out of bounds [0,'
                             +str(len(self.segments)-1)+']: '
                             +str(index))
        mapped_segment = []
        for offset in [0, 2]:
            mapped_segment += self.map_point(self.segments[index][offset],
                                             self.segments[index][offset+1])
        return tuple(mapped_segment)

    def has_intersection_end(self, index, reverse=False):
        if index < 0 or index > len(self.segments)-1:
            raise ValueError('index out of bounds [0,'
                             +str(len(self.segments)-1)+']: '
                             +str(index))
        offset = 0
        if reverse:
            offset = -2
        number_adjacent = 0
        for jj in range(len(self.segments)):
            if index == jj:
                continue
            if (self.segments[index][(2+offset):(2+offset+2)] == self.segments[jj][:2]
                or self.segments[index][(2+offset):(2+offset+2)] == self.segments[jj][2:]):
                number_adjacent += 1
                if number_adjacent >= 2:
                    return True
        return number_adjacent >= 2

    def has_intersection_start(self, index):
        return self.has_intersection_end(index, reverse=True)

    def plot(self, ax):
        import matplotlib as mpl
        for idx in range(self.number_of_segments()):
            road = self.get_mapped_segment(idx)
            ax.plot([road[0], road[2]],
                    [road[1], road[3]], 'ko-')


def road_segment(x1, x2, prefix='straightroad',
                 x1_intersection=False, x2_intersection=False):
    assert len(x1) == 2 and len(x2) == 2

    center = ((x1[0] + x2[0])/2.0, (x1[1] + x2[1])/2.0)
    length = math.sqrt((x1[0]-x2[0])**2 + (x1[1]-x2[1])**2)
    if x1_intersection and x2_intersection:
        lane_marker_center = center
        lane_marker_length = length-1.2
    elif x1_intersection and (not x2_intersection):
        lane_marker_length = length-0.6+0.06
        lane_marker_center = (x2[0] + (lane_marker_length/2.0 - 0.06)*(x1[0]-x2[0])/length,
                              x2[1] + (lane_marker_length/2.0 - 0.06)*(x1[1]-x2[1])/length)
    elif (not x1_intersection) and x2_intersection:
        lane_marker_length = length-0.6+0.06
        lane_marker_center = (x1[0] + (lane_marker_length/2.0 - 0.06)*(x2[0]-x1[0])/length,
                              x1[1] + (lane_marker_length/2.0 - 0.06)*(x2[1]-x1[1])/length)
    else:
        lane_marker_center = center
        lane_marker_length = length+0.12
    angle = math.atan2(x2[1]-x1[1], x2[0]-x1[0])
    nl = '\n'
    idt = ' '*2
    output = '<model name="'+prefix+'">'+nl
    output += '<static>true</static>'+nl
    output += '<link name="'+prefix+'link1">'+nl
    output += '<visual name="'+prefix+'bar1">'+nl
    output += '<pose>{X} {Y} 0 0 0 {ANGLE}</pose>'.format(X=center[0], Y=center[1], ANGLE=angle)+nl
    output += '<geometry><box><size>{LENGTH} 1.2 0.001</size></box></geometry>'.format(LENGTH=length+1.2)+nl
    output += """
          <material>
	    <ambient>0.2 0.2 0.2 1</ambient>
	    <diffuse>0.2 0.2 0.2 1</diffuse>
	    <specular>0.2 0.2 0.2 1</specular>
	    <emissive>0.2 0.2 0.2 1</emissive>
	  </material>
	</visual>
      </link>
"""
    output += '<link name="'+prefix+'link2">'+nl
    output += '<visual name="'+prefix+'bar2">'+nl
    output += '<pose>{X} {Y} 0 0 0 {ANGLE}</pose>'.format(X=lane_marker_center[0], Y=lane_marker_center[1], ANGLE=angle)+nl
    output += '<geometry><box><size>{LENGTH} 0.12 0.0015</size></box></geometry>'.format(LENGTH=lane_marker_length)+nl
    output += """
          <material>
	    <ambient>0.5 0.5 0 1</ambient>
	    <diffuse>0.5 0.5 0 1</diffuse>
	    <specular>0.5 0.5 0 1</specular>
	    <emissive>0.5 0.5 0 1</emissive>
	  </material>
	</visual>
      </link>
"""
    return output+nl+'</model>'

def gen_worldsdf(roads):
    output = """<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <gui><camera name="user_camera"><pose>0 0 9.5 0 1.5 0</pose></camera></gui>

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
"""
    for sindex in range(roads.number_of_segments()):
        segment = roads.get_mapped_segment(sindex)
        output += road_segment((segment[0], segment[1]),
                               (segment[2], segment[3]),
                               prefix='segment_'+str(sindex)+'_',
                               x1_intersection=roads.has_intersection_start(sindex),
                               x2_intersection=roads.has_intersection_end(sindex))
    output += '</world></sdf>'
    return output


class Problem(object):
    """Problem instance of the domain: traffic network of Dubins cars
    """
    def __init__(self):
        self.goals = []
        self.rnd = None
        self.intersection_radius = 0.0

    @staticmethod
    def loadJSONdict(probd):
        prob = Problem()
        assert probd['version'] == 0
        prob.goals = probd['goals']
        prob.rnd = RoadNetwork(probd['rnd'])
        prob.intersection_radius = probd['intersection_radius']
        return prob

    @staticmethod
    def loadJSON(probjs):
        return Problem.loadJSONdict(json.loads(probjs))
