import numpy as np
from pydrake.multibody.plant import CoulombFriction
from pydrake.math import (RotationMatrix, RigidTransform)
from pydrake.geometry import (
        Box, ProximityProperties, AddContactMaterial, AddRigidHydroelasticProperties
    )


def add_plate(plant, inclined_plane_angle=0.0, origin=[0,0,0], plate_length=15.0, plate_width=15.0, visible=True, friction=0.2, name="InclinedPlaneVisualGeometry"):
    """ 
    Create a flat plate using a box shape. Accomplished by registering a box visual and collision geometry to the plant.
    NOTE: if adding multple plates, you need to specify unique names

    @param plant_setupplant: the plant to add the plate to
    @param inclined_plane_angle: [degrees] angle about the y axis to set the plane
    @param origin: location of the center of the plate
    @param plate_length: [m] set the length of the box
    @param plate_width: [m] set the width of the plate
    @param visible: set to false if you dont want to see the plant
    @param friction: set efault friction settings dyn and static fric as treated the same here
    @param name: need to supply another value if registering more than one plane to the plant

    @return: the modified plant
    """

    
    plate_height = 0.05  # Height of the plate (thickness)
    stc_fric = friction
    dyn_fric = friction
    green = np.array([0.5, 1.0, 0.5, 1.0])

    coefficient_friction_inclined_plane = CoulombFriction(stc_fric, dyn_fric)
    proxProperties = ProximityProperties()
    AddRigidHydroelasticProperties(0.01, proxProperties)
    AddContactMaterial(None, None, coefficient_friction_inclined_plane, proxProperties)
    R_WA = RotationMatrix.MakeYRotation(inclined_plane_angle *np.pi/180)
    # Set inclined plane A's visual geometry and collision geometry to a
    # box whose top surface passes through world origin Wo.  To do this,
    # set Ao's position from Wo as -0.5 * LAz * Az (half-width of box A).
    Az_W = R_WA.col(2);  #Az in terms of Wx, Wy, Wz.
    p_WoAo_W = origin - 0.5 * plate_height * Az_W
    X_WA  = RigidTransform(R_WA, p_WoAo_W)
    if visible:
        plant.RegisterVisualGeometry(plant.world_body(), X_WA, Box(plate_length, plate_width, plate_height),
                                    name,
                                    green)
    plant.RegisterCollisionGeometry(plant.world_body(), X_WA, Box(plate_length, plate_width, plate_height),
                                     name,
                                     proxProperties)

    return plant
