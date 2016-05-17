import json
import numpy as np

# inline:
#   matplotlib
#   cdd


class Polytope(object):
    """

    Initialize with H-representation. Generate V-representation when requested.
    """
    def __init__(self, H, K):
        self.H = H.copy()
        self.K = K.copy()
        self.V = None

    def contains(self, x):
        if (np.dot(self.H, x) <= self.K).all():
            return True
        else:
            return False

    def cache_clear(self):
        self.V = None

    def getVrep(self, force=False):
        """

        If force is True, then do not use cached result if
        available. This option provides a way to skip memoization and
        is equivalent to calling cache_clear() and then getVrep().
        """
        import cdd
        if (not force) and (self.V is not None):
            return self.V.copy()
        Hmat = cdd.Matrix(np.hstack((self.K.reshape(self.K.shape[0], 1), -self.H)), number_type='float')
        Hmat.rep_type = cdd.RepType.INEQUALITY
        P = cdd.Polyhedron(Hmat)
        Vrep = P.get_generators()
        self.V = np.array(Vrep)[:,1:]
        return self.V.copy()

    def get_bbox(self):
        """Get bounding box of polytope.

        The bounding box is the minimal axis-aligned rectangle
        containing the polytope. It is represented as a 1-D NumPy
        ndarray of the form [x1_min, x1_max, x2_min, x2_max, ...].
        """
        V = self.getVrep()
        mins = np.min(V, axis=0)
        maxs = np.max(V, axis=0)
        return np.array(zip(mins, maxs)).flatten()

    def plot(self, ax, alpha=1.0, color=None):
        """

        Derived from plotting routines in the Python polytope package
        <https://pypi.python.org/pypi/polytope>.
        """
        import matplotlib as mpl
        if color is None:
            color = np.random.random_sample(3)
        ax.add_patch(mpl.patches.Polygon(self.getVrep(), closed=True,
                                         facecolor=color, alpha=alpha,
                                         edgecolor='black', linewidth=3))

class LabeledPolytope(Polytope):
    def __init__(self, H, K, label=""):
        self.label = label
        Polytope.__init__(self, H, K)


class Problem(object):
    """Problem instance of the domain: scaling chains of integrators
    """
    def __init__(self):
        self.Xinit = None
        self.goals = []
        self.obstacles = []
        self.Y = None
        self.U = None
        self.output_dim = None
        self.number_integrators = None

    @staticmethod
    def loadJSONdict(probd):
        prob = Problem()
        assert probd['version'] == 0
        prob.Xinit = np.array(probd['Xinit'])
        prob.Y = Polytope(np.array(probd['Y']['H']), np.array(probd['Y']['K']))
        prob.U = Polytope(np.array(probd['U']['H']), np.array(probd['U']['K']))
        prob.goals = [LabeledPolytope(np.array(goaldict['H']),
                                      np.array(goaldict['K']),
                                      label=goaldict['label'])
                      for goaldict in probd['goals']]
        prob.obstacles = [LabeledPolytope(np.array(obsdict['H']),
                                          np.array(obsdict['K']),
                                          label=obsdict['label'])
                          for obsdict in probd['obstacles']]
        if 'period' in probd:
            prob.period = probd['period']
        else:
            prob.period = None
        prob.output_dim = prob.Y.H.shape[1]
        prob.number_integrators = prob.Xinit.shape[0]/prob.output_dim
        return prob

    @staticmethod
    def loadJSON(probjs):
        return Problem.loadJSONdict(json.loads(probjs))

    def plot(self, ax, outdims=None):
        if outdims is None:
            outdims = [0, 1]
        self.Y.plot(ax, color=(1,1,1))
        ax.hold(True)
        for lpoly in self.goals+self.obstacles:
            lpoly.plot(ax, alpha=0.8)
            xc = np.mean(lpoly.getVrep(), axis=0)
            ax.text(xc[0], xc[1], lpoly.label)
        ax.plot(self.Xinit[outdims[0]], self.Xinit[outdims[1]], 'o')
