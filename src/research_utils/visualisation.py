# FT 13/8/26

import numpy as np
from matplotlib import colors
from mpl_toolkits.mplot3d import art3d

from research_utils.transforms import calc_body_to_inertial, ned_to_xyz


class Plane:
    # '_xyz' indicates that the position coordinates are in the xyz frame
    def __init__(
        self,
        ax,
        scale=1,
        init_x_xyz=0,
        init_y_xyz=0,
        init_z_xyz=0,
        init_phi=0,
        init_theta=0,
        init_psi=0,
        colour=None,
        linewidth=None,
        zorder=1e5,
    ):
        # https://stackoverflow.com/questions/4622057/plotting-3d-polygons
        # WOT4: wingspan 1.33m, length 1.2m
        v = scale * np.array(
            [
                [1.5, 0, 0],
                [-1, 1, 0],
                [-0.5, 0, -0.3],
                [-1, -1, 0],
            ]
        )
        f = [[0, 1, 2], [0, 2, 3]]
        # Done like this so that the code in set_pose can still rotate the vertices.
        self.verts = np.array([[v[i] for i in p] for p in f])
        self.verts_flat = self.verts.reshape(np.prod(self.verts.shape[:2]), 3)
        self.plane = art3d.Poly3DCollection(self.verts)
        self.plane.set_zorder(zorder)
        # Each plane will be a random colour if not specified
        self.plane.set_color(colour if colour else colors.rgb2hex(np.random.rand(3)))
        self.plane.set_edgecolor("k")
        if linewidth is not None:
            self.plane.set_linewidth(linewidth)

        ax.add_collection3d(self.plane)

        # Set initial position and orientation
        self.set_pose(
            init_x_xyz, init_y_xyz, init_z_xyz, init_phi, init_theta, init_psi
        )

    def transform_verts(self, verts):
        return verts.reshape(*self.verts.shape[:2], 3)

    def set_pose(self, posnx_xyz, posny_xyz, posnz_xyz, phi, theta, psi):
        # Orient
        ## Body to inertial matrix calculates the inertial frame coordinates of a vector expressed in the body
        ## frame. Alternatively, it can be used to rotate a vector in the fixed inertial perspective by the
        ## orientation of the body frame. This is what it is being used for here - to align the aircraft polygon
        ## with the body frame.
        # Using this matrix, we can treat the aircraft vertices as if they are in the body frame, and then find their inertial frame positions for plotting.
        rotat = calc_body_to_inertial(phi, theta, psi)
        # verts needs to be in the format...
        #  +-                    -+
        #  |   |    |    |        |
        #  |  pt1  pt2  pt3  ...  |
        #  |   |    |    |        |
        #  +-                    -+
        # so that each vertex point is rotated by the rotation matrix. This is why self.verts
        # is transposed, since this is the opposite orientation to that required by Poly3DCollection.
        oriented_verts = np.matmul(rotat, self.verts_flat.T)

        # Convert NED system coordinates (for *plane vertices* - position of plane already given in xyz coord system) to
        # xyz coord system used by Matplotlib for plotting.
        # (Need to flip E and D)
        # [TODO Is this true, following code modifications?] NOTE The oriented_verts array is changed by the ned_to_xyz() function, so it technically doesn't have to be returned and re-assigned, but it helps to make
        # the code more maintainable by making value changes more obvious.
        # TODO [Do a proper comment for this] Want to use the scalar version so that it comes out as a 2D Numpy array. Note that the expansion operator (*)
        # has to be used in the argument.
        oriented_verts = ned_to_xyz(*oriented_verts)

        # Translate
        # posn_x_xyz, posn_y_xyz and posn_z_xyz should be in the xyz coordinate system used by Matplotlib for plotting.
        shifted_verts = oriented_verts + np.array(
            [[posnx_xyz], [posny_xyz], [posnz_xyz]]
        )

        # .T to transpose back to the expected format.
        self.plane.set_verts(self.transform_verts(shifted_verts.T))
