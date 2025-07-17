import numpy as np
from pydrake.systems.framework import LeafSystem
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

class ballDynamics_2D():
    ''' copied from the jumping conditions memo'''
    def __init__(self):
        
        self.state = "rolling"

        self.m_pe = 22.03
        self.m_pc = 2.78
        self.m_shell = 16.54
        self.m_total = self.m_pc + self.m_shell + self.m_pe
        self.m_p = self.m_pc+self.m_pe
        self.r = 0.095
        self.Iz_pc = 0.00754
        self.Iz_pe = 0.4988
        self.Iz_shell = 0.64074
        self.g = 9.81



    def M(self, q):
        t_g = q[2] + q[3]
        mt = self.m_total 
        alpha = (self.m_p)*self.r*np.cos(t_g)
        beta = (self.m_p)*self.r*np.sin(t_g)

        return np.array([[mt, 0, alpha, alpha],
                         [0, mt, beta, beta],
                         [alpha, beta, self.Iz_pc+self.Iz_pe+self.Iz_shell+self.m_pe*self.r**2, self.m_pe*self.r**2],
                         [alpha, beta, self.m_pe*self.r**2, self.Iz_pe+ self.m_pe*self.r**2]])

    def H(self, q):
        q1 = q[0:4]
        
        dq = q[4:]
        t_g = q1[2] + q1[3]
        dt_g = dq[2] + dq[3]
        alpha = (self.m_p) * self.r * np.cos(t_g)
        beta = (self.m_p) * self.r * np.sin(t_g)
        return np.array([[-beta*dt_g**2],
                         [alpha*dt_g**2 + self.m_total*self.g],
                         [dt_g*(alpha*dq[1] - beta*dq[0]) + beta*self.g],
                         [dt_g*(alpha*dq[1] - beta*dq[0]) ]])

class Pravecek_2025_Model(LeafSystem):
    '''
    Derived from Equation (8) in:
    D. J. Pravecek et al. "Empirically Compensated Setpoint Tracking for Spherical Robots 
    With Pressurized Soft-Shells," in IEEE Robotics and Automation Letters,
    vol. 10, no. 3, pp. 2136-2143, March 2025

    All defined on LHS such as: 
    calc_M() @ \ddot{q} + calc_C() @ \dot{q} + G() = U^T \\tau_m

    Note the paper defines the pipe angle in an
    '''
    def __init__(self):
        super().__init__()

        self.g = 9.81
        ''' params copied from URDF file for steer direction'''
        #masses
        self.m_pc = 2.77934 # [kg] mass of pitch_center
        self.m_p = 22.03    # [kg] mass of pendulm
        self.m_s = 16.53649 # [kg] mass of pipe assembly
        print('total prav mass: ', self.m_pc + self.m_p + self.m_s)
        # geometry
        self.r_pc = 0.05    # [m] COM of pitch_center
        self.r_p = 0.095   # [m] COM of pendulum
        self.R = 0.3175      # [m] outer radius of bed_collision spherical mesh

        # inertias
        self.I_s = 1.04     # [kgm^2] Ix == Iz for shell or pipe_assembly
        self.I_pc = 0.01841 # [kgm^2] Ix for pitch_center
        self.I_p = 0.44235  # [kgm^2] Ix for pendulum

        # drake input and output ports
        self.DeclareContinuousState(4)
        self.torque_input_port = self.DeclareVectorInputPort("tau_s", 1)
        self.get_state_output_port = self.DeclareVectorOutputPort("prav_state_output", 4, self.CalcModelResponse, {self.all_state_ticket()})

    def calc_M(self, phi, theta_g):
        I_eq = self.I_s + self.I_pc + self.m_pc*self.r_pc**2 + (self.m_s + self.m_pc + self.m_p)*self.R**2 - 2*self.m_pc*self.r_pc*self.R*np.cos(phi)
        M11 = I_eq
        M12 = -self.m_p*self.r_p*self.R*np.cos(theta_g)
        M22 = self.I_p+self.m_p*self.r_p**2

        return np.array([[M11, M12],
                         [M12, M22]])
    def calc_C(self, phi, theta_g, phi_dot, theta_g_dot):
        C1 = self.m_pc*self.r_pc*self.R*np.sin(phi)*phi_dot
        C2 = self.m_p*self.r_p*self.R*np.sin(theta_g)*theta_g_dot
        return np.array([[C1, C2],
                         [ 0,  0]])

    def calc_G(self, phi, theta_g):
        return np.array([[self.m_pc*self.r_pc*self.g*np.sin(phi)],
                         [self.m_p*self.r_p*self.g*np.sin(theta_g)]])
    
    def DoCalcTimeDerivatives(self, context, derivatives):
        # q of the form: phi, theta_g, phi_dot, theta_g_dot
        
        q = context.get_continuous_state_vector().CopyToVector()
        tau_s = self.torque_input_port.Eval(context)
        phi = q[0]
        phi_dot = q[2]
        theta_g = q[1]
        theta_g_dot = q[3]
        q_dot = np.array([[phi_dot],[theta_g_dot]])

        m_inv = np.linalg.inv(self.calc_M(phi, theta_g))
        c_q = self.calc_C(phi, theta_g, phi_dot, theta_g_dot) @ q_dot
        v_q = self.calc_G(phi, theta_g)
        U_tau_m = np.array([[1], [-1]])*tau_s
        q_ddot = m_inv @ (U_tau_m-c_q-v_q)
        # print(np.vstack((q_dot,q_ddot)))
        derivatives.get_mutable_vector().SetFromVector(np.vstack((q_dot,q_ddot)))

    def CalcModelResponse(self, context, output):

        q = context.get_continuous_state_vector().CopyToVector()
        q[1] = -q[0] + q[1] # theta = phi - theta_g
        q[3] = -q[2] + q[3]
        output.SetFromVector(q)


class FingerprintData():
    def __init__(self):
        # Data as class variables, shared by all instances
        self.pressure = np.array([2, 2.5, 3, 3.5, 4.0])

        self.pipe_angle_4psi = np.array([-19.1, 6.3, -3.8, -13.9, -9, 1.3, 21.7, 16.6, 11.5, -24.1])*np.pi/180
        self.pend_angle_4psi = np.array([9.333333333, -1.2, 6.266666667, 10.4, 9.6, 2.533333333, -5.2, -5.733333333, -5.066666667, 7.733333333,])*np.pi/180

        self.pipe_angle_3_5psi = np.array([10.7, 5.45, 0.4, 15.9, 21.1, -10.05, -20.5, -4.9, -15.3])*np.pi/180
        self.pend_angle_3_5psi = np.array([-9.333333, -1.666667, 7.4666667, -11.2, -10.667, 17.1334, 13.0667, 14.2667, 15.8667])*np.pi/180

        self.pipe_angle_3psi = np.array([0.6, -24.4, -9.2, 21, -19.4, -14.3, -4.1, 5.8, 10.9, 15.9])*np.pi/180
        self.pend_angle_3psi = np.array([3.86667, 16, 17.2, -12.133333, 16.66667, 18, 12.266667, -5.4666667, -11.6, -12.93333])*np.pi/180

        self.pipe_angle_2_5psi = np.array([24.2, -22.6, 3.6, -17.4, -12.2, -7.1, -1.8, 3.5, 8.6, 13.9, 19.1])*np.pi/180
        self.pend_angle_2_5psi = np.array([-15.73333333, 18.26666667, -0.266666667, 20.26666667, 20.13333333, 17.33333333, 10.13333333, -0.266666667, -11.6, -16.26666667, -16.26666667])*np.pi/180

        self.pipe_angle_2psi = np.array([ -12.3, -8.6, -4.8, -1, 2.7, 6.3, 10.2, 14, 17.8, 21.4])*np.pi/180
        self.pend_angle_2psi = np.array([ 23.73333333, 21.2, 16.4, 9.6, -1.6, -11.2, -15.6, -18.4, -19.33333333, -20])*np.pi/180


class PravModelWithEmpiricalShellFingerprint(FingerprintData, Pravecek_2025_Model):
    
    def __init__(self):
        FingerprintData.__init__(self)
        Pravecek_2025_Model.__init__(self)
        self.phi_grid = None
        self.p_grid   = None
        self.predicted_angles = None
        self.y_predicted = None
        self.r_squared = None
        self.popt = None
        self.pipe_angles_range =  None
        self.pressures_range = None
        self.set_pressure = 3.0 # default pressure
        self.already_warned_about_pressure_input = False
        X = []
        y = []
        # Prepare the data for fitting on instatiation
        for i, pipe_angle in enumerate([self.pipe_angle_2psi, self.pipe_angle_2_5psi, self.pipe_angle_3psi, self.pipe_angle_3_5psi, self.pipe_angle_4psi]):
            for angle in pipe_angle:
                X.append([angle, self.pressure[i]])

        for angle in [self.pend_angle_2psi, self.pend_angle_2_5psi, self.pend_angle_3psi, self.pend_angle_3_5psi, self.pend_angle_4psi]:
            for pend_angle in angle:
                y.append(pend_angle)

        self.X = np.array(X)
        self.y = np.array(y)

        # Ensure X and y have the same number of entries
        if len(X) != len(y):
            raise ValueError("Mismatch in number of entries: X has {}, y has {}".format(len(X), len(y)))
        self.ff_curve_fit() # build the regressions

    def derek_formula_func(self, X, a1, b1, a2, b2, a3, b3, c3, d3, o):
        # Define the derek formula function
        phi, p = X
        A = a1 * p + b1
        B = a2 * p + b2
        C = a3 * p**3 + b3 * p**2 + c3 *p + d3

        return A * np.sin(B * np.tanh(C*(phi - o))) + o

    def compute_r_squared(self, y_actual, y_predicted):
        # Function to compute R^2
        mean_actual = np.mean(y_actual)
        ss_res = np.sum((y_actual - y_predicted) ** 2)
        ss_tot = np.sum((y_actual - mean_actual) ** 2)
        r_squared = 1 - (ss_res / ss_tot)
        return r_squared

    def ff_curve_fit(self):
        # Curve fitting
        initial_guess = [0.4173, -2.7654, -0.0306, 0.271, -1.4313, 11.941, -31.852, 34.668, -0.0349]
        self.popt, _ = curve_fit(self.derek_formula_func, self.X.T, self.y, p0=initial_guess, maxfev=1000000)

        # Generate a grid of pipe angles and pressures
        self.pipe_angles_range = np.linspace(-30*np.pi/180, 30*np.pi/180, 1000)  # Adjust as needed
        self.pressures_range = np.linspace(2, 4, 1000)  # Adjust as needed
        self.phi_grid, self.p_grid = np.meshgrid(self.pipe_angles_range, self.pressures_range)

        # Compute predicted values on the grid
        self.predicted_angles = self.derek_formula_func(np.vstack([self.phi_grid.ravel(), self.p_grid.ravel()]), *self.popt).reshape(self.phi_grid.shape)


        self.y_predicted = self.derek_formula_func(self.X.T, *self.popt)

        # Compute R^2
        self.r_squared = self.compute_r_squared(self.y, self.y_predicted)

        # Print R^2 value
        print(f"Total R^2: {self.r_squared:.4f}")

    def plot_surfaces(self):
        # Colors for each pressure level
        colors = ['C0', 'C1', 'C2', 'C3', 'C4']

        # Plotting
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Surface plot
        ax.plot_surface(self.phi_grid*180/np.pi, self.p_grid, self.predicted_angles*180/np.pi, cmap='viridis', edgecolor='none', alpha=0.5)

        self.pipe_angle_4psi*=180/np.pi
        self.pipe_angle_3_5psi*=180/np.pi
        self.pipe_angle_3psi*=180/np.pi
        self.pipe_angle_2_5psi*=180/np.pi
        self.pipe_angle_2psi*=180/np.pi

        self.pend_angle_4psi*=180/np.pi
        self.pend_angle_3_5psi*=180/np.pi
        self.pend_angle_3psi*=180/np.pi
        self.pend_angle_2_5psi*=180/np.pi
        self.pend_angle_2psi*=180/np.pi
        # Scatter plot for actual data
        for i in range(len(self.pressure)):
            ax.scatter(
                [self.pipe_angle_2psi, self.pipe_angle_2_5psi, self.pipe_angle_3psi, self.pipe_angle_3_5psi, self.pipe_angle_4psi][i],
                [self.pressure[i]] * len([self.pipe_angle_2psi, self.pipe_angle_2_5psi, self.pipe_angle_3psi, self.pipe_angle_3_5psi, self.pipe_angle_4psi][i]),
                [self.pend_angle_2psi, self.pend_angle_2_5psi, self.pend_angle_3psi, self.pend_angle_3_5psi, self.pend_angle_4psi][i],
                color=colors[i], label=str(self.pressure[i])+ ' psi', s=50
            )

        ax.set_xlabel('Pipe Angle (deg)')
        ax.set_ylabel('Pressure (psi)')
        ax.set_zlabel('Global Pendulum Angle (deg)')
        ax.set_title('Pressure Dependent S$^3$MF Fitted Global Pendulum Angle \n at Different Pressures with Total R² = '+str(round(self.r_squared,3)))
        plt.legend()
        plt.show()

        print("Optimal parameters:", self.popt)


        Pipe_Angle = [self.pipe_angle_2psi, self.pipe_angle_2_5psi, self.pipe_angle_3psi, self.pipe_angle_3_5psi, self.pipe_angle_4psi]
        Pend_Angle = [self.pend_angle_2psi, self.pend_angle_2_5psi, self.pend_angle_3psi, self.pend_angle_3_5psi, self.pend_angle_4psi]


        i = 0
        while i < len(self.pressure):
            pipe_angle = Pipe_Angle[i]
            pend_angle = Pend_Angle[i]
            p = self.pressure[i]
            X = self.pipe_angles_range, p
            pend_model = self.derek_formula_func(X, *self.popt)

            plt.scatter(pipe_angle, pend_angle, color=colors[i])
            plt.plot(self.pipe_angles_range*180/np.pi, pend_model*180/np.pi, color=colors[i], label=str(p)+' psi')

            i+=1

        plt.xlabel('Pipe Angle (deg)')
        plt.ylabel('Global Pendulum Angle (deg)')
        plt.title('Fitted Global Pendulum Angle with R² = '+str(round(self.r_squared,3)))
        plt.grid()
        plt.minorticks_on()
        plt.legend()
        plt.show()


        # from the prav_2025_model class
        mpc = self.m_pc
        rpc = self.r_pc
        mp =  self.m_p
        rp =  self.r_p

        g = self.g

        i = 0
        while i < len(self.pressure):
            pipe_angle = Pipe_Angle[i]
            pend_angle = Pend_Angle[i]
            p = self.pressure[i]
            X = self.pipe_angles_range, p
            pend_model = self.derek_formula_func(X, *self.popt)

            tau_data = -mpc*rpc*g*np.sin(pipe_angle*np.pi/180) - mp*rp*g * np.sin(pend_angle*np.pi/180)
            tau_model = -mpc*rpc*g*np.sin(self.pipe_angles_range) - mp*rp*g * np.sin(pend_model)
            # self.tau_ff(self.pipe_angles_range, pend_model)
            tau_pitch_center = mpc*rpc*g*np.sin(np.linspace(-30*np.pi/180, 30*np.pi/180, 1000))
            # plt.scatter(pipe_angle, tau_data, color=colors[i])
            plt.plot(self.pipe_angles_range*180/np.pi, tau_model, color=colors[i], label=str(p)+' psi')
            i+=1
            
        plt.plot(np.linspace(-30*np.pi/180, 30*np.pi/180, 1000)*180/np.pi, tau_pitch_center, label='$m_{pc} r_{pc} g \sin(\phi)$', color='gold')
        plt.xlabel('Pipe Angle (deg)')
        plt.ylabel('Flat Torque (N)')
        plt.title('Fitted Soft-Shell Magic Formula Torque')
        plt.grid()
        plt.minorticks_on()
        plt.legend()
        plt.show()

    
    def tau_ff(self, input_pipe_angle, input_pressure=None):
        if input_pressure==None:
            if not self.already_warned_about_pressure_input:
                print(f"no pressure input, using default class pressure of {self.set_pressure}")
                self.already_warned_about_pressure_input = True
            input_pressure = self.set_pressure
        X = input_pipe_angle, input_pressure
        pend_model = self.derek_formula_func(X, *self.popt)
        tau_model = -self.m_pc*self.r_pc*self.g*np.sin(input_pipe_angle) - self.m_p*self.r_p*self.g * np.sin(pend_model)
        
        return tau_model
    
    def DoCalcTimeDerivatives(self, context, derivatives): # should overide prav class methods
        # q of the form: phi, theta_g, phi_dot, theta_g_dot
        
        q = context.get_continuous_state_vector().CopyToVector()
        tau_s = self.torque_input_port.Eval(context)
        phi = q[0]
        phi_dot = q[2]
        theta_g = q[1]
        theta_g_dot = q[3]
        q_dot = np.array([[phi_dot],[theta_g_dot]])

        m_inv = np.linalg.inv(self.calc_M(phi, theta_g))
        c_q = self.calc_C(phi, theta_g, phi_dot, theta_g_dot) @ q_dot
        v_q = self.calc_G(phi, theta_g)

        tau_s = self.torque_input_port.Eval(context)
        U_tau_m = np.array([[1], [-1]])*tau_s
        tau_flat = np.array([[1], [0]]) * self.tau_ff(phi) # map the flat torque to the pipe EOM
        q_ddot = m_inv @ (U_tau_m-c_q-v_q - tau_flat)
       
        
        derivatives.get_mutable_vector().SetFromVector(np.vstack((q_dot,q_ddot)))
    
if __name__=="__main__":
    # testing
    fingerprint = PravModelWithEmpiricalShellFingerprint()

    print("Called w 2psi", fingerprint.tau_ff(np.deg2rad(-45), 2.0))
    print("called w 3 psi", fingerprint.tau_ff(np.deg2rad(-45), 3.0))
    print('called with default of 3', fingerprint.tau_ff(np.deg2rad(-45)))
    fingerprint.set_pressure = 2.0
    print("called w defult set to 2", fingerprint.tau_ff(np.deg2rad(-45)))