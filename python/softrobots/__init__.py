# -*- coding: utf-8 -*-
"""
The SoftRobots Template Library.
================================

Utility functions and scene templates for the real-time simulation framework `Sofa <https://www.sofa-framework.org/>`_
and the `SoftRobots <https://project.inria.fr/softrobot/>`_ plugin.

The library can be used with scenes written in python and `PSL <https://github.com/sofa-framework/sofa/tree/master/applications/plugins/PSL>`_.

Example:
********

.. sourcecode:: python

    from stlib.scene import MainHeader
    from stlib.physics.rigid import Cube, Floor
    from stlib.physics.deformable import ElasticMaterialObject

    from softrobots.actuators import PullingCable
    from softrobots.sensors import StringSensor

    def createScene(rootNode):
        MainHeader(rootNode)
        DefaultSolver(rootNode)

        Cube(rootNode, translation=[5.0,0.0,0.0])
        Floor(rootNode, translation=[0.0,-1.0,0.0])

       target = ElasticMaterialObject(fromVolumeMesh="mesh/liver.msh",
                                       withTotalMass=0.5,
                                       attachedTo=node)

        PullingCable(target)
        StringSensor(target)

Content of the library
**********************

.. autosummary::
    :toctree: _autosummary

    softrobots.actuators
    softrobots.sensors

Indices and tables
******************

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

"""

__all__=["actuators", "sensors"]

