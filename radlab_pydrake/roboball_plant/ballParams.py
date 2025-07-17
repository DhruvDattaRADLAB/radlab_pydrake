class RoboBall2Params():
    """
    Important Measurable Values for the Ball Plant
    NOT found in the URDF
    """
    def __init__(self):
        self.steer_dynamic_friction = 0.7
        self.steer_static_friction = 0.65
        self.steer_viscous_damping = 0.104

        # robot urdf
        self.package_path = "./roboball_plant/RoboBall_URDF/package.xml"
        self.robot_file = "./roboball_plant/RoboBall_URDF/urdf/RoboBall_URDF.urdf"
        self.lumpy_robot_file = "./roboball_plant/RoboBall_URDF/urdf/RoboBall_URDF_lumpy.urdf"
        self.shell_file = "./roboball_plant/RoboBall_URDF/urdf/RoboBall_shell.urdf"


        #actuator properties
        self.steerGearRatio = 9.0 * 50.0 / 30.0
        self.driveGearRatio = 9.0 * 3.0 * 50.0 / 30.0

        '''
        rotor inertias from dereks notepad

        I_sp = 2.08e-6 kg*m2
        I_sh = 2.53e-6 kg*m2
        I_mo = 9.42e-4 + 2.7e-6 kgm2 <- assume this is good enough
        '''
        self.NeoRotorInertia =  9.42e-4