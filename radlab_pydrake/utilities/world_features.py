import numpy as np
from pydrake.multibody.plant import CoulombFriction
from pydrake.math import (RotationMatrix, RigidTransform)
from pydrake.geometry import (
        Box, ProximityProperties, AddContactMaterial, AddRigidHydroelasticProperties
    )
from pydrake.multibody.math import SpatialForce
from pydrake.systems.framework import LeafSystem
from pydrake.common.value import Value
from pydrake.common.cpp_param import List
from pydrake.multibody.plant import ExternallyAppliedSpatialForce_
from pydrake.common.eigen_geometry import Quaternion

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

def add_ramp_to_plant(
        plant,
        origin,
        length,
        height,
        width,
        offset_height,
        theta=0.28,
        mu_s=0.65,
        mu_k=0.10):
    """
    This will load in an angled ramp to the simulation

    @param plant: plant to load the sim into
    @param @dhruv take it from here
    """
    print(f"Building ramp at {np.rad2deg(theta)} degrees")

    mass = 1.0
    inertia = UnitInertia.SolidBox(length, width, height)
    spatial_inertia = SpatialInertia(mass=mass, p_PScm_E=[0, 0, 0], G_SP_E=inertia)

    ramp_body = plant.AddRigidBody("ramp", spatial_inertia)

    box_shape = Box(length, width, height)

    X_WR = RigidTransform(
        RollPitchYaw(0, theta, 0),
        [origin[0], origin[1], origin[2] + offset_height]
    )

    plant.WeldFrames(plant.world_frame(), ramp_body.body_frame(), X_WR)

    plant.RegisterCollisionGeometry(
        ramp_body, RigidTransform(), box_shape, "ramp_collision", CoulombFriction(mu_s, mu_k)
    )

    plant.RegisterVisualGeometry(
        ramp_body, RigidTransform(), box_shape, "ramp_visual", np.array([0.6, 0.4, 0.2, 1.0])
    )

class YawConstraint(LeafSystem):
    def __init__(self, plant, dissipation_param=0.4, q0=0.0):
        # applies a force about the global vertial to constrain the global yaw motion

        LeafSystem.__init__(self)
        forces_cls = Value[List[ExternallyAppliedSpatialForce_[float]]]
        self.DeclareVectorInputPort("yaw_velocity", 1)
        self.DeclareAbstractOutputPort("spatial_forces",
                                        lambda: forces_cls(),
                                        self.CalcDisturbances)
        self.plant = plant
        self.pole_body = self.plant.GetBodyByName("bedliner")
        self.dissipation = dissipation_param
        # to integrate the velocity
        self.t_step = 0.001
        self.wx_sum = q0
        

    def CalcDisturbances(self, context, spatial_forces_vector):
        # Apply a force at COM of the Pole body.
        wx = self.GetInputPort("yaw_velocity").Eval(context)[0] # extract from vector

        yaw_force = -self.dissipation*wx #- self.stiffness*self.wx_sum 
        self.wx_sum += wx # integrate wx
        force = ExternallyAppliedSpatialForce_[float]()
        force.body_index = self.pole_body.index()
        force.p_BoBq_B = self.pole_body.default_com()
        # print("com_of_body", force.p_BoBq_B)
        spatial_force = SpatialForce(
            tau=[0, 0, yaw_force],
            f=[0, 0, 0])
        
        force.F_Bq_W = spatial_force
        spatial_forces_vector.set_value([force])

class ShellStiffness(LeafSystem):
    def __init__(self, plant, stiffness_param=0.0):
        # applies a force about the global vertial to constrain the global yaw motion

        LeafSystem.__init__(self)
        forces_cls = Value[List[ExternallyAppliedSpatialForce_[float]]]
        self.DeclareVectorInputPort("plant_states", plant.num_positions() + plant.num_velocities())
        self.DeclareAbstractOutputPort("spatial_forces",
                                        lambda: forces_cls(),
                                        self.CalcDisturbances)
        self.plant = plant
        self.shell = self.plant.GetBodyByName("bedliner")
        self.K_r = np.diag([stiffness_param, 0, 0])
        
        self.vn_frame = plant.GetFrameByName("vn_frame")

    def CalcDisturbances(self, context, spatial_forces_vector):
        # Apply a force at COM of the Pole body.
        q = self.GetInputPort("plant_states").Eval(context)
        plant_context = self.plant.CreateDefaultContext()
        self.plant.SetPositionsAndVelocities(plant_context, q)
        
        X_WB = self.vn_frame.CalcPoseInWorld(plant_context) 
        R_WB = X_WB.rotation()
        quat = R_WB.ToQuaternion()
        # quat = q[0:4] / np.linalg.norm(q[0:4])
        # quat = Quaternion(quat) # the first four terms are the quaternion

        # Relative rotation from upright = identity rotation
        # Compute angle vector using RollPitchYaw
        R = RotationMatrix(quat).matrix()
        pipe_angle = np.arcsin(R[2, 1])
        rpy = [pipe_angle, 0,0 ]
        # Assume small angles → torque = -K * angle deviation
        torque_W = self.K_r @ rpy
        torque_B = R_WB@torque_W
        force = ExternallyAppliedSpatialForce_[float]()
        force.body_index = self.shell.index()
        force.p_BoBq_B = self.shell.default_com()
        # print(dir(force))
        # print("com_of_body", force.p_BoBq_B)
        spatial_force = SpatialForce(
            tau=[torque_B[0], torque_B[1], torque_B[2]],
            f=[0, 0, 0])
        
        force.F_Bq_W = spatial_force
        
        spatial_forces_vector.set_value([force])