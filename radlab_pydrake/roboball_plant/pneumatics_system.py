from pydrake.systems.framework import LeafSystem
import numpy as np

class DynamicPressureModel(LeafSystem):
    """
        Implements the RoboBall Pressure Control System 

        @params manual: if True, compressor and solenoid control ports are delared with 0-off and 1-on
                        if False, a setpoint input point is delcared

        in both cases the state output is given by
        @params OutputPort: self.get_tank_ball_pressure --> [P_tank, P_ball]
        if manual True
        @params InputPort: 0 or 1 --> self.get_solenoid_control
        @params InputPort: 0 or 1 --> self.get_compressor_control
        if not manual
        @params InputPort: double --> self.pressure_setpoint
    """
    def __init__(self, manual=False):
        super().__init__()

        # declare two states for pressure system
        self.DeclareContinuousState(2)
        #set up the outputs
        self.get_tank_ball_pressure = self.DeclareVectorOutputPort("tank_ball_pressure", 2, self.calc_out_pressure)
        
        #if manual, take compressor and solenoid inputs
        if manual:
            self.get_solenoid_control = self.DeclareVectorInputPort("solenoid_boolean", 1)
            self.get_compressor_control = self.DeclareVectorInputPort("compressor_boolean", 1)
        if not manual: # then require a ball setpoint
            self.pressure_setpoint = self.DeclareVectorInputPort("pressure_setpoint_psi", 1)

        # system states
        self.u_c = 0 # both off to start
        self.u_s = 0

        self.manual = manual
        self.t_off = 0 # tracking variable for time
        ####### system parameters #######
        # Constants for the model of the system
        #Leak constants
        self.kt = 0.001095878 * 0.85
        self.kb = 0.00013587
        self.Pc = 2.1366 *1.1
        self.Ps = 0.008

        #Volume ratio calculation
        self.vBall = 0.85 * (4 / 3) * (12 ** 3) * np.pi  # volume of the ball accounting for space taken up (in^3)
        self.vTanks = 574 * 3 / 16.387  # volume of pressure tanks converted to (in^3)
        self.vRatio = self.vTanks / self.vBall
        #Control Constants
        self.alpha_t_c = 2.1366*1.1
        self.alpha_b_c = 0.0446
        self.alpha_t_s = 0.05791 * 1.3
        self.alpha_b_s = 0.004943 / 3.6

    def bang_bang(self, Pt, Pb, context):
        setpoint = self.pressure_setpoint.Eval(context)
        t = context.get_time()
        tol = 0.1 # psi
        out = np.array([[0], [0]])
        e = Pb - setpoint[0]
        if abs(self.t_off - t) > 5: # only make a control action every 5 seconds

            if e < 0 - tol:
                if Pb < setpoint + tol:  # only turn on solenoid when pressure is less than required
                    out = np.array([[1], [0]])
                   
            elif e > 0 - tol:
                if Pt > setpoint - tol:  # only turn on compressor when pressure is less than required
                    out = np.array([[0], [1]])
                   
            else:
                out = np.array([[0], [0]])
                self.t_off = t
                
        return out
    
    def DoCalcTimeDerivatives(self, context, derivatives):
        # defines this LeafSystem as dynamic
        P = context.get_continuous_state_vector().CopyToVector()
        Pt = P[0]
        Pb = P[1]
        # get the input from port
        if self.manual:
            self.u_c = self.get_compressor_control.Eval(context)
            self.u_s = self.get_solenoid_control.Eval(context)
            u_control = np.array([self.u_c, self.u_s])
        if not self.manual:
            u_control = self.bang_bang(Pt, Pb, context)


        A = np.array([[-self.kt, self.kt],
                      [self.vRatio*self.kt, -self.kb - self.vRatio*self.kt]])
        B = np.array([[self.alpha_t_c, -self.alpha_t_s*(Pt - Pb)],
                      [-self.alpha_b_c, self.alpha_b_s*(Pt - Pb)]])
        
     
        pdot = A @ np.array([[Pt], [Pb]]) + B @ u_control # u doesnt neet brackets because it is a vector numpy interprest this as vertical then

        derivatives.get_mutable_vector().SetFromVector(pdot)

    def calc_out_pressure(self, context, output):
        q = context.get_continuous_state_vector().CopyToVector()
        # return all the states
        output.SetFromVector(q)

if __name__=="__main__":
    # debug the pressure model class in  the drake sim
    from pydrake.all import (DiagramBuilder, Simulator, ConstantVectorSource, LogVectorOutput, DiscreteTimeDelay)
    import matplotlib.pyplot as plt

    # Create a simple block diagram containing our system
    manual = True

    builder = DiagramBuilder()
    if manual:
        sys = DynamicPressureModel(manual)
        mySys = builder.AddSystem(sys)  # add system to diagram
        zeroSignal = builder.AddSystem(ConstantVectorSource([0])) # add a step input controller to the diagram
        onesSignal = builder.AddSystem(ConstantVectorSource([1])) # add a step input controller to the diagram
        
        discTimeDelay = builder.AddSystem(DiscreteTimeDelay(10, 1, 1)) # delay 2.5 seconds
        # builder.Connect(stepInput.get_output_port(0), discTimeDelay.get_input_port(0))  # connect the step input to the time delay
        builder.Connect(onesSignal.get_output_port(0), discTimeDelay.get_input_port(0))
        builder.Connect(discTimeDelay.get_output_port(0), mySys.get_solenoid_control)
        builder.Connect(zeroSignal.get_output_port(0), mySys.get_compressor_control)
    if not manual:
        sys = DynamicPressureModel(manual)
        mySys = builder.AddSystem(sys)  
        setpoint_2 = builder.AddSystem(ConstantVectorSource([1.25]))
    
        builder.Connect(setpoint_2.get_output_port(), mySys.pressure_setpoint)


    # connect output of delayed step with system dynamics
    logger_tank_output = LogVectorOutput(mySys.get_tank_ball_pressure, builder)
    # logger_input = LogVectorOutput(discTimeDelay.get_output_port(0), builder)
    diagram = builder.Build()

    # set the ICs
    context = diagram.CreateDefaultContext()

    Pt0 = 100 #psi
    Pb0 = 1.5 #psi

    context.SetContinuousState([Pt0, Pb0])

    # creat the simulator
    simulator = Simulator(diagram, context)
    simulator.AdvanceTo(20)

    # Grab output results from Logger:
    tank_log = logger_tank_output.FindLog(context) # find output log with that context
    tank_time = tank_log.sample_times()
    tank_data = tank_log.data().transpose()

   
    fig, ax = plt.subplots()
    ax2 = ax.twinx()
    plt.title('Example system using pydrake')
    ax.plot(tank_time, tank_data[:, 0], label='Pt')
    ax2.plot(tank_time, tank_data[:, 1], label="Pb")
    plt.legend()
    plt.grid()
    plt.show()