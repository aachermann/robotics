import numpy as np
from roboticstoolbox import DHRobot, RevoluteMDH
from spatialmath import SE3


class myRobot(DHRobot):
    """
    Class that models a Lynxmotion myRobot manipulator

    :param symbolic: use symbolic constants
    :type symbolic: bool

    ``myRobot()`` is an object which models a Lynxmotion myRobot robot and
    describes its kinematic and dynamic characteristics using modified DH
    conventions.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.DH.myRobot()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration

    .. note::
        - SI units are used.

    :References:

        - 'Reference of the robot <http://www.lynxmotion.com/c-130-AL5D.aspx>'_

    .. codeauthor:: Tassos Natsakis
    """  # noqa

    def __init__(self, symbolic=False):

        if symbolic:
            import spatialmath.base.symbolic as sym

            # zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi

            # zero = 0.0
        
        deg = pi / 180
        # robot length values (metres)
        a = [0, 0, -0.56036, -0.62662]
        d = [0.208, 0, 0, 0]

        alpha = [0, 90*deg, 0, -90*deg]
        offset = [0, 0, 0, 0]

        # mass data as measured
        # mass = [0.187, 0.044, 0.207, 0.081]

        # center of mass as calculated through CAD model
        center_of_mass = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
        ]

        # moments of inertia are practically zero
        moments_of_inertia = [
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
        ]

        joint_limits = [
            [0 , 90*deg ],
            [90*deg , 270*deg ],
            [-175*deg , 0 ],
            [0 , 0 ],
        ]

        links = []

        for j in range(4):
            link = RevoluteMDH(
                d=d[j],
                a=a[j],
                alpha=alpha[j],
                offset=offset[j],
                #r=center_of_mass[j],
                #I=moments_of_inertia[j],
                G=1,
                B=0,
                Tc=[0, 0],
                qlim=joint_limits[j],
            )
            links.append(link)

        tool = SE3(0, 0, 0)

        super().__init__(
            links,
            name="myRobot",
            manufacturer="PRENGruppe13",
            keywords=("dynamics", "symbolic"),
            symbolic=symbolic,
            tool=tool,
        )

        # zero angles
        self.q = np.array([0, 265*deg, -165*deg, 0])
        self.addconfiguration("q", self.q)
        self.home = np.array([0, 265*deg, -165*deg, 0])
        self.addconfiguration("home", self.home)
        self.home2 = np.array([0, 190*deg, -30*deg, 0])
        self.addconfiguration("home2", self.home2)
        self.home3 = np.array([10*deg, 190*deg, -30*deg, 0])
        self.addconfiguration("home3", self.home3)



if __name__ == "__main__":  # pragma nocover

    myrobot = myRobot(symbolic=False)
    print(myRobot)