from numpy.linalg import norm
from numpy import deg2rad, asarray, pi
from pydrake.multibody.parsing import Parser
from pydrake.multibody.tree import PlanarJoint, FixedOffsetFrame, RevoluteJoint
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.common.eigen_geometry import Quaternion

from pydrake.geometry import RoleAssign, ProximityProperties
from pydrake.math import RollPitchYaw, RotationMatrix, RigidTransform

from .ballParams import RoboBall2Params
from ament_index_python.packages import get_package_share_directory
import os



def add_RoboBall_plant(plant, place_in_stand=False, lumpy_bedliner=False):
    """
        @brief: THIS WILL FINALIZE THE PLANT

        @param plant: the plant object to load the urdf into
        @param place_in_static_stand: options are ("hanging", "drive", "steer")
        
        "hanging": the returned plant will swap the pitch_center for the pipe_assembly
        as the top of the kinematic tree. The pipe will then be welded to the world frame. 
        NOTE: This swaps the actuator order so use their names to index. 
            
        "drive": this will add a planar joint in the drive direction of the ball. the steer
        actuator will be removed and locked

        "steer": this will add a planar joint in the steer direction of the ball. the drive
        actuator will be removed and locked. 

        After loading in the plant in the desired configuration, the system default gear ratios
        and rotor inertias will be set if the respective actuator still exists. 

        The plant will be finalized upon return. So add in any other world features (ramps, obstacles, etc)
        before you call this.

    """

    # check validity of flags
    if place_in_stand not in ("hanging", "drive", "steer", False):
        raise ValueError("Invalid Option for RoboBall Plant Load in")
    
    # instantiate the params
    ballParams = RoboBall2Params()

    if lumpy_bedliner:
        print("loading lumpy bedliner shell")
        file_path = ballParams.lumpy_robot_file
    else:
        file_path = ballParams.robot_file
    # load in the ball

    # Prepare to load the robot model
    model_file_url = (
        'package://roboball_urdf/urdf/'
        'RoboBall_URDF.urdf')
    model_name = "RoboBall"

    parser = Parser(plant)
    pkg_map = parser.package_map()
    pkg_map.PopulateFromRosPackagePath()

    model_index = parser.AddModels(url=model_file_url) 
    
    if len(model_index) == 1:
        model_index = model_index[0]

    if place_in_stand == "hanging":
        # remove the drive joint
        plant.RemoveJointActuator(plant.GetJointActuatorByName("drive"))
        plant.RemoveJoint(plant.GetJointByName("drive"))

        # get frames to restructure
        pipe_frame = plant.GetFrameByName("pipe_assembly")
        pitch_frame = plant.GetFrameByName("pitch_center")
        # add reversed joint with actuator
        new_joint = plant.AddJoint(RevoluteJoint("drive", 
                                     frame_on_parent=pipe_frame, 
                                     frame_on_child=pitch_frame,
                                     axis=[0,1,0]))
        plant.AddJointActuator(name='drive', joint=new_joint)

        pipe_body = plant.GetBodyByName("pipe_assembly")
        
        # this will face the robot towards the user in meshcat
        X_stand = RigidTransform(
            RollPitchYaw(asarray([0, 0, 90]) * pi / 180), p=[0,0,0])
        # welds the base link to float in air
        plant.WeldFrames(plant.world_frame(), pipe_body.body_frame(), X_stand) 

    elif place_in_stand in ("drive", "steer"):
        # this will isolate the drive and steer directions of the ball
        pitch_frame = plant.GetFrameByName("pitch_center")
        world_frame = plant.world_frame()

        # Planar joints must work in a xy frame
        # Define a rotations to make reference frame F and M such that the approriate direction is constrained
        # see "drake::multibody::PlanarJoint" for more info
        # https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_planar_joint.html
        
        R_AB = RotationMatrix.MakeXRotation(deg2rad(90))
        if place_in_stand == "steer":
            # if its steer add an extra rotation to align the Z-axis out front
            R_AB = R_AB.multiply(RotationMatrix.MakeYRotation(deg2rad(90)))
        
        rotated_world_frame_F = FixedOffsetFrame(
            name="rotated_frame",
            P=world_frame,  # Parent frame
            X_PF=RigidTransform(R_AB)      # The transform from parent frame to the new frame
        )

        # R_AB_pc = RotationMatrix.MakeXRotation(deg2rad(90))
        rotated_pitch_frame_M = FixedOffsetFrame(
            name="rotated_frame_pc",
            P=pitch_frame,  # Parent frame
            X_PF=RigidTransform(R_AB)      # The transform from parent frame to the new frame
        )

        # Register the frame with the plant
        planar_P = plant.AddFrame(rotated_world_frame_F)
        planar_body = plant.AddFrame(rotated_pitch_frame_M)
        # plant.WeldFrames(world_frame, rot_frame)

        # constrain to be planar
        planar_joint = PlanarJoint(
            name="planar_joint",
            frame_on_parent=planar_P,
            frame_on_child=planar_body, 
        )

        # Add the joint to the plant
        plant.AddJoint(planar_joint)

        if place_in_stand == "drive":
            # lock the steer axis
            plant.RemoveJointActuator(plant.GetJointActuatorByName("steer"))
            plant.RemoveJoint(plant.GetJointByName("steer"))
            plant.WeldFrames(plant.GetFrameByName("pitch_center"), plant.GetFrameByName("pendulum"))
        elif place_in_stand == "steer":
            # lock the drive axis
            plant.RemoveJointActuator(plant.GetJointActuatorByName("drive"))
            plant.RemoveJoint(plant.GetJointByName("drive"))
            plant.WeldFrames(plant.GetFrameByName("pitch_center"), plant.GetFrameByName("pipe_assembly"))


    plant.Finalize()

    # add reflected inertias and joint reductions
    if plant.HasJointActuatorNamed("steer"):
        steer_actuator = plant.GetJointActuatorByName("steer")
        steer_actuator.set_default_gear_ratio(ballParams.steerGearRatio)
        # steer_actuator.set_default_rotor_inertia(ballParams.NeoRotorInertia)
    if plant.HasJointActuatorNamed("drive"):
        drive_actuator = plant.GetJointActuatorByName("drive")
        drive_actuator.set_default_gear_ratio(ballParams.driveGearRatio)
        # drive_actuator.set_default_rotor_inertia(ballParams.NeoRotorInertia)
    
    
    return plant, model_index

def add_planar_roboball(plant, direction="drive"):
    """
        DEPREACIATED: use add_ball_plant anf the test stand flag instead
        @brief: THIS WILL FINALIZE THE PLANT but will constrain the ball to only move in the drive direction

        use meshcat.Set2dRenderMode() to display only the planar case

        @param plant: the plant object to load the urdf into
        @param direction: "drive" or "steer" render the ball in either direction
    """
    # instantiate the params
    ballParams = RoboBall2Params()
    # load in the ball
    parser = Parser(plant)
    pkg_map = parser.package_map()
    pkg_map.AddPackageXml(ballParams.package_path)
    model_index = parser.AddModels(ballParams.robot_file) 
    
    if len(model_index) == 1:
        model_index = model_index[0]
    
    pitch_frame = plant.GetFrameByName("pitch_center")
    world_frame = plant.world_frame()

    # Planar joints must work in a xy frame
    # Define a rotations to make reference frame F and M such that the approriate direction is constrained
    # see "drake::multibody::PlanarJoint" for more info
    # https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_planar_joint.html
    
    R_AB = RotationMatrix.MakeXRotation(deg2rad(90))
    if direction == "steer":
        # if its steer add an extra rotation to align the Z-axis out front
        R_AB = R_AB.multiply(RotationMatrix.MakeYRotation(deg2rad(90)))
    
    rotated_world_frame_F = FixedOffsetFrame(
        name="rotated_frame",
        P=world_frame,  # Parent frame
        X_PF=RigidTransform(R_AB)      # The transform from parent frame to the new frame
    )

    # R_AB_pc = RotationMatrix.MakeXRotation(deg2rad(90))
    rotated_pitch_frame_M = FixedOffsetFrame(
        name="rotated_frame_pc",
        P=pitch_frame,  # Parent frame
        X_PF=RigidTransform(R_AB)      # The transform from parent frame to the new frame
    )

    # Register the frame with the plant
    planar_P = plant.AddFrame(rotated_world_frame_F)
    planar_body = plant.AddFrame(rotated_pitch_frame_M)
    # plant.WeldFrames(world_frame, rot_frame)

    # constrain to be planar
    planar_joint = PlanarJoint(
        name="planar_joint",
        frame_on_parent=planar_P,
        frame_on_child=planar_body, 
    )

    # Add the joint to the plant
    plant.AddJoint(planar_joint)

    if direction == "drive":
        # lock the steer axis
        plant.RemoveJointActuator(plant.GetJointActuatorByName("steer"))
        plant.RemoveJoint(plant.GetJointByName("steer"))
        plant.WeldFrames(plant.GetFrameByName("pitch_center"), plant.GetFrameByName("pendulum"))
    elif direction == "steer":
        # lock the drive axis
        plant.RemoveJointActuator(plant.GetJointActuatorByName("drive"))
        plant.RemoveJoint(plant.GetJointByName("drive"))
        plant.WeldFrames(plant.GetFrameByName("pitch_center"), plant.GetFrameByName("pipe_assembly"))

    plant.Finalize()
    return plant, model_index

def add_RoboBall_shell(plant, lumpy_bedliner=False):
    """
        @brief: THIS WILL FINALIZE THE PLANT

        Loads in a stripped down urdf of just the pipe_assembly and bedliner for simple testing

        @param plant: the plant object to load the urdf into
    """
    # instantiate the params
    ballParams = RoboBall2Params()
    # load in the ball
    parser = Parser(plant)
    pkg_map = parser.package_map()
    pkg_map.AddPackageXml(ballParams.package_path)

    if lumpy_bedliner:
        print("loading lumpy bedliner shell")
        model_index = parser.AddModels(ballParams.lumpy_shell_file)
    else:
        model_index = parser.AddModels(ballParams.shell_file) 
    
    if len(model_index) == 1:
        model_index = model_index[0]

    plant.Finalize()

    return plant, model_index

def update_bedliner_properties(scene_graph, new_properties):
    '''
    @brief: Updates the bedliner contact parameters, should work on all types of plant load ins. Call 
    just after the robot is loaded in

    @param scene_graph: SceneGraph object the ball is attached to
    @param new_preperties: a list of new preperty values [modulus, dissipation]
    '''
    # update proximitiy properties
    inspector = scene_graph.model_inspector()

    sourceIds = inspector.GetAllSourceIds()
    plant_id_idx = [inspector.GetName(x) for x in sourceIds].index('plant')
    bedliner_source_id = sourceIds[plant_id_idx]
    geoIds = inspector.GetAllGeometryIds()
    # print("Geometry IDs: ", geoIds, "\n")
    # print("Output of GetNAme: \n", [inspector.GetName(x) for x in geoIds])

    # find the ID for the bedliner TODO: is there a better way?
    bedliner_collision_idx = [inspector.GetName(x) for x in geoIds].index(f"roboball_urdf::bed_collision")
    bedliner_collision_id = geoIds[bedliner_collision_idx]


    # get current properties
    old_props = inspector.GetProximityProperties(bedliner_collision_id)

    # print("Current Dissipation: \n", old_props.GetProperty("material", "hunt_crossley_dissipation"))
    # print("Current HydroElastic Modulus \n", old_props.GetProperty("hydroelastic", "hydroelastic_modulus"))
    # print("Current Mesh Hint \n", old_props.GetProperty("hydroelastic", "resolution_hint"))


    new_props =  ProximityProperties(old_props)
    new_props.UpdateProperty("material", "hunt_crossley_dissipation", new_properties[1])
    new_props.UpdateProperty("hydroelastic", "hydroelastic_modulus", new_properties[0])
    new_props.UpdateProperty("hydroelastic", "resolution_hint", 0.1)
    scene_graph.AssignRole(bedliner_source_id, bedliner_collision_id, new_props,
                            RoleAssign(1))

    # check the values
    old_props = inspector.GetProximityProperties(bedliner_collision_id)

    print("New Dissipation: \n", old_props.GetProperty("material", "hunt_crossley_dissipation"))
    print("New HydroElastic Modulus \n", old_props.GetProperty("hydroelastic", "hydroelastic_modulus"))
    print("New Mesh Hint \n", old_props.GetProperty("hydroelastic", "resolution_hint"))

if __name__=="__main__":
    # prototyping

    from pydrake.all import StartMeshcat,SceneGraphConfig,AddMultibodyPlant,MultibodyPlantConfig
    meshcat = StartMeshcat()


    builder = DiagramBuilder()
    sceneConfig = SceneGraphConfig()
    sceneConfig.default_proximity_properties.compliance_type = "compliant"
    plant, scene_graph = AddMultibodyPlant(
        MultibodyPlantConfig(
            time_step=0.001,
            penetration_allowance=0.001,
            contact_surface_representation="polygon",
            contact_model="hydroelastic"
            ), sceneConfig, 
        builder)

    # plant = add_plate(plant, 0.0, visible=True)

    plant, model_idx = add_RoboBall_plant(plant, place_in_static_stand=True)