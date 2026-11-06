import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from IPython.display import HTML
import matplotlib.patches as patches
import plotly.graph_objects as go


def Rrpy(phi, theta, psi):
    """
    Devuelve la matriz de rotación a partir de los ángulos de Euler phi, theta, psi
    tal que R = Rz(phi) * Ry(theta) * Rx(psi), donde phi=roll, theta=pitch, psi=yaw
    """
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(phi), -np.sin(phi)],
                   [0, np.sin(phi),  np.cos(phi)]])

    Ry = np.array([[np.cos(theta), 0, np.sin(theta)],
                   [0, 1, 0],
                   [-np.sin(theta), 0, np.cos(theta)]])

    Rz = np.array([[np.cos(psi), -np.sin(psi), 0],
                   [np.sin(psi), np.cos(psi), 0],
                   [0,0,1]])
    R = Rz @ Ry @ Rx
    return R


class RobotAereo3D:
    """
    Simula la dinámica de un robot aéreo con 4 rotores en 3D. Las entradas
    son las velocidades de giro de los motores w1, w2, w3, w4
    """

    def __init__(self):
        """
        Parámetros iniciales
        """
        self.m  = 0.5        # Masa
        self.Ix = 0.1        # Momento de inercia alrededor del eje x
        self.Iy = 0.1        # Momento de inercia alrededor del eje y
        self.Iz = 0.2        # Momento de inercia alrededor del eje z
        self.l = 0.566/(2*np.sqrt(2))     # Longitud del centro al motor
        self.kf = 1.0; self.km = 1.0      # Constantes
        self.g = 9.81

        # Estado: [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
        #                   roll, pitch, yaw                     wx, wy, wz
        self.X = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def set_state(self, X):
        """
        Establecer el estado del robot
        """
        self.X = X

    def get_state(self):
        """
        Recuperar el estado del robot
        """
        return self.X

    def set_control(self, w1, w2, w3, w4):
        """
        Establecer las velocidades angulares de los motores
        """
        self.w1 = w1
        self.w2 = w2
        self.w3 = w3
        self.w4 = w4

    def aceleracion_lineal(self):
        """
        Dinámica traslacional
        """
        R = Rrpy(self.phi, self.theta, self.psi)
        g = np.array([0, 0, self.g])
        ftotal = self.kf * (self.w1**2 + self.w2**2 + self.w3**2 + self.w4**2)
        aceleraciones =  g + 1/self.m * R@np.array([0, 0, -ftotal])
        return aceleraciones

    def get_rpy_derivatives(self):
        """
        Obteniene derivadas de RPY usando las velocidades angulares p, q, r
        """
        # Derivadas de los ángulos RPY usando
        M = np.array([[1, np.sin(self.phi)*np.tan(self.theta), np.cos(self.phi)*np.tan(self.theta)],
                      [0, np.cos(self.phi), -np.sin(self.phi)],
                      [0, np.sin(self.phi)/np.cos(self.theta), np.cos(self.phi)/np.cos(self.theta)]])
        # Velocidad angular en el sistema del cuerpo
        pqr = np.array([self.p, self.q, self.r])
        # Derivadas de Roll, Pitch, Yaw
        return M @ pqr

    def get_angular_acceleration(self):
        """
        Retorna las aceleraciones angulares en el sistema del cuerpo (dp, dq, dr)
        """
        tau_x = self.kf*self.l*(self.w1**2 - self.w2**2 - self.w3**2 + self.w4**2)
        tau_y = self.kf*self.l*(self.w1**2 + self.w2**2 - self.w3**2 - self.w4**2)
        tau_z = self.km * (self.w1**2 - self.w2**2 + self.w3**2 - self.w4**2)

        p_dot = 1/self.Ix * (tau_x - self.q*self.r*(self.Iz - self.Iy))
        q_dot = 1/self.Iy * (tau_y - self.p*self.r*(self.Ix - self.Iz))
        r_dot = 1/self.Iz * (tau_z - self.p*self.q*(self.Iy - self.Ix))
        return np.array([p_dot, q_dot, r_dot])

    def simulate(self, dt):
        """
        Simula el movimiento del robot
        """
        # Incrementar el estado
        drpy = self.get_rpy_derivatives()           # droll, dpitch, dyaw
        dpqr = self.get_angular_acceleration()      # dp, dq, dr
        acc = self.aceleracion_lineal()             # ddx, ddy, ddz
        X_dot = np.array([self.X[6], self.X[7], self.X[8],
                          drpy[0], drpy[1], drpy[2],
                          acc[0], acc[1], acc[2],
                          dpqr[0], dpqr[1], dpqr[2]])
        # Integración de Euler
        self.X = self.X + X_dot * dt
        return self.X

    @property
    def phi(self): return self.X[3]

    @property
    def theta(self): return self.X[4]

    @property
    def psi(self): return self.X[5]

    @property
    def p(self): return self.X[9]

    @property
    def q(self): return self.X[10]

    @property
    def r(self): return self.X[11]


def convert_control_to_motors(u1, ux, uy, uz, m, L, kf, km):
    w1 = np.sqrt(-m*u1/(4*kf) + uz/(4*km) + ux/(4*L*kf) + uy/(4*L*kf))
    w2 = np.sqrt(-m*u1/(4*kf) - uz/(4*km) - ux/(4*L*kf) + uy/(4*L*kf))
    w3 = np.sqrt(-m*u1/(4*kf) + uz/(4*km) - ux/(4*L*kf) - uy/(4*L*kf))
    w4 = np.sqrt(-m*u1/(4*kf) - uz/(4*km) + ux/(4*L*kf) - uy/(4*L*kf))
    return w1, w2, w3, w4


class AnimacionRobot3D:
    def __init__(self):
        # Parámetros del quadcopter
        self.arm_length = 0.5   # Longitud de los brazos
        self.prop_radius = 0.1  # Radio de las hélices

        # Configurar la figura
        self.fig = plt.figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Listas para almacenar la trayectoria
        self.trajectory_x = []
        self.trajectory_y = []
        self.trajectory_z = []

    def rotation_matrix(self, roll, pitch, yaw):
        """
        Calcula la matriz de rotación R = Rot_z(yaw) * Rot_y(pitch) * Rot_x(roll)
        """
        # Matriz de rotación en X (roll)
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

        # Matriz de rotación en Y (pitch)
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])

        # Matriz de rotación en Z (yaw)
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])

        # Rotación compuesta: R = Rz * Ry * Rx
        return Rz @ Ry @ Rx

    def draw_quadcopter(self, position, orientation, color='red'):
        """
        Dibuja el quadcopter en una posición y orientación específicas
        """
        x, y, z = position
        roll, pitch, yaw = orientation

        # Obtener matriz de rotación
        R = self.rotation_matrix(roll, pitch, yaw)

        # Definir la geometría del quadcopter en el marco local
        # Brazos del quadcopter (en forma de X)
        arm1 = np.array([self.arm_length, 0, 0])
        arm2 = np.array([0, self.arm_length, 0])
        arm3 = np.array([-self.arm_length, 0, 0])
        arm4 = np.array([0, -self.arm_length, 0])

        # Rotar los brazos según la orientación
        arm1_rot = R @ arm1
        arm2_rot = R @ arm2
        arm3_rot = R @ arm3
        arm4_rot = R @ arm4

        # Posiciones de los motores
        motor_positions = np.array([arm1_rot, arm2_rot, arm3_rot, arm4_rot])

        # Dibujar el marco del quadcopter
        # Brazo 1 (centro a motor 1)
        self.ax.plot([x, x + arm1_rot[0]], [y, y + arm1_rot[1]],
                    [z, z + arm1_rot[2]], color=color, linewidth=3)

        # Brazo 2 (centro a motor 2)
        self.ax.plot([x, x + arm2_rot[0]], [y, y + arm2_rot[1]],
                    [z, z + arm2_rot[2]], color=color, linewidth=3)

        # Brazo 3 (centro a motor 3)
        self.ax.plot([x, x + arm3_rot[0]], [y, y + arm3_rot[1]],
                    [z, z + arm3_rot[2]], color=color, linewidth=3)

        # Brazo 4 (centro a motor 4)
        self.ax.plot([x, x + arm4_rot[0]], [y, y + arm4_rot[1]],
                    [z, z + arm4_rot[2]], color=color, linewidth=3)

        # Dibujar los motores como puntos
        for motor_pos in motor_positions:
            self.ax.scatter(x + motor_pos[0], y + motor_pos[1], z + motor_pos[2],
                           c='black', s=50, marker='o')

        # Dibujar el centro del quadcopter
        self.ax.scatter(x, y, z, c=color, s=100, marker='s')

        # Dibujar vector de dirección frontal
        front_vector = R @ np.array([0.3, 0, 0])  # Vector frontal
        self.ax.plot([x, x + front_vector[0]], [y, y + front_vector[1]],
                    [z, z + front_vector[2]], color='blue', linewidth=2, alpha=0.8)

        return motor_positions

    def animate(self, frame, trajectory_data):
        """
        Función de animación para cada frame
        """
        self.ax.clear()

        # Obtener datos del frame actual
        pos = trajectory_data['position'][frame]
        orient = trajectory_data['orientation'][frame]

        # Agregar posición actual a la trayectoria
        self.trajectory_x.append(pos[0])
        self.trajectory_y.append(pos[1])
        self.trajectory_z.append(pos[2])

        # Dibujar la trayectoria
        if len(self.trajectory_x) > 1:
            self.ax.plot(self.trajectory_x, self.trajectory_y, self.trajectory_z,
                        'b-', alpha=0.5, linewidth=1, label='Trayectoria')

        # Dibujar el quadcopter
        self.draw_quadcopter(pos, orient, color='red')

        # Configurar la visualización
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title(f'Quadcopter - Frame {frame} | '
                    f'Pos: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) | '
                    f'R:{np.degrees(orient[0]):.0f}° P:{np.degrees(orient[1]):.0f}° Y:{np.degrees(orient[2]):.0f}°',
                    fontsize=10, pad=10)

        # Establecer límites de los ejes
        max_range = 4
        self.ax.set_xlim([-max_range, max_range])
        self.ax.set_ylim([-max_range, max_range])
        self.ax.set_zlim([max_range/2, -max_range])   # Invertir eje debido a que Z es positivo hacia abajo

        # Agregar grid y leyenda solo si hay trayectoria
        self.ax.grid(True, alpha=0.3)
        if len(self.trajectory_x) > 1:
            self.ax.legend()

        # Configurar vista
        self.ax.view_init(elev=20, azim=45)  # Sin rotación de la cámara

    def run_simulation(self, trajectory_data=None, interval=100, save_gif=False):
        """
        Ejecuta la simulación completa
        """

        # Limpiar trayectoria anterior
        self.trajectory_x = []
        self.trajectory_y = []
        self.trajectory_z = []

        n_frames = len(trajectory_data['time'])

        # Crear animación
        anim = FuncAnimation(self.fig, self.animate, frames=n_frames,
                             fargs=(trajectory_data,), interval=interval, repeat=True)

        # Cerrar la figura para evitar que se muestre estáticamente
        plt.close(self.fig)
        # Para Google Colab, mostrar como HTML
        plt.tight_layout()

        return HTML(anim.to_jshtml())


def plot_trajectory(traj_x, traj_y, traj_z, step=5):
    """
    Crear visualización 3D interactiva con Plotly
    """
    # Crear figura
    fig = go.Figure()

    # Agregar trayectoria principal
    fig.add_trace(go.Scatter3d(x=traj_x, y=traj_y, z=traj_z,
                               mode='lines',
                               line=dict(color='blue', width=4),
                               name='Trayectoria',
                               hovertemplate='X: %{x:.2f}<br>Y: %{y:.2f}<br>Z: %{z:.2f}<extra></extra>'
    ))

    # Puntos de inicio y fin
    fig.add_trace(go.Scatter3d(x=[traj_x[0]], y=[traj_y[0]], z=[traj_z[0]],
                               mode='markers',
                               marker=dict(color='green', size=8, symbol='circle'),
                               name='Inicio'
    ))

    fig.add_trace(go.Scatter3d(x=[traj_x[-1]], y=[traj_y[-1]], z=[traj_z[-1]],
                               mode='markers',
                               marker=dict(color='red', size=8, symbol='square'),
                               name='Fin'
    ))

    # Configuración del layout
    fig.update_layout(
        title='Trayectoria 3D Interactiva con Orientación<br>',
        scene=dict(xaxis_title='X (m)',
                   yaxis_title='Y (m)',
                   zaxis_title='Z (m)',
                   zaxis=dict(autorange='reversed'),  # Invertir el eje Z
                   aspectmode='cube',  # Mantener proporciones
            camera=dict(eye=dict(x=1.5, y=1.5, z=1.5))
        ),
        width=900,
        height=700
    )

    return fig