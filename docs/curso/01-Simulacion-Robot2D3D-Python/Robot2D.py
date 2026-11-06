
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from IPython.display import HTML


class RobotAereo2D:
    """
    Simula la dinámica de un robot aéreo en 2D restringido al movimiento
    en el plano yz. Las entradas son u1, u2
    """
    def __init__(self, Ix=0.1, m=0.2):
        """
        Parámetros iniciales
        """
        self.Ix = Ix     # Momento de inercia alrededor del eje x
        self.m = m       # Masa
        self.u1 = 0.0    # Empuje (thrust) total
        self.u2 = 0.0    # Momento alrededor del eje x
        self.g = 9.81    # Aceleración de la gravedad
        # Estado inicial: y, z, phi, dy, dz, dphi
        self.X = np.array([0., 0., 0., 0., 0., 0.]) 
    
    def advance_state(self, dt):
        """
        Incrementar el estado del robot
        """
        phi = self.X[2]
        
        # Dinámica del robot
        ddy = np.sin(phi)*self.u1/self.m
        ddz = self.g - self.u1*np.cos(phi)/self.m
        ddphi = self.u2/self.Ix
        
        # Ecuación en variables de estado
        dX = np.array([self.X[3], self.X[4], self.X[5], 
                       ddy, ddz,  ddphi])        
        # Integración usando Euler
        self.X = self.X + dX*dt
        
        return self.X

    def set_state(self, X):
        """
        Establecer el estado del robot
        """
        self.X = X
    
    def set_control(self, u1, u2):
        """
        Establecer las señales de control u1, u2
        """
        self.u1 = u1
        self.u2 = u2


def create_robot_shape(y, z, phi, size=0.1):
    """
    Crea la forma visual del robot (cuerpo + hélices)
    """
    # Cuerpo principal (rectángulo)
    body_width = size
    body_height = size * 0.3
    
    # Rotación del cuerpo
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    
    # Vértices del cuerpo en coordenadas locales
    body_local = np.array([
        [-body_width/2, -body_height/2],
        [body_width/2, -body_height/2],
        [body_width/2, body_height/2],
        [-body_width/2, body_height/2]
    ])
    
    # Rotar y trasladar
    body_global = np.zeros_like(body_local)
    for i in range(4):
        body_global[i, 0] = cos_phi * body_local[i, 0] - sin_phi * body_local[i, 1] + y
        body_global[i, 1] = sin_phi * body_local[i, 0] + cos_phi * body_local[i, 1] + z
    
    # Posiciones de las hélices
    prop_offset = size * 0.6
    prop1_local = np.array([-prop_offset, 0])
    prop2_local = np.array([prop_offset, 0])
    
    prop1_global = np.array([cos_phi * prop1_local[0] - sin_phi * prop1_local[1] + y,
                             sin_phi * prop1_local[0] + cos_phi * prop1_local[1] + z
    ])
    
    prop2_global = np.array([cos_phi * prop2_local[0] - sin_phi * prop2_local[1] + y,
                             sin_phi * prop2_local[0] + cos_phi * prop2_local[1] + z
    ])
    
    return body_global, prop1_global, prop2_global


def animate_robot_from_data(y, z, phi, y_des=None, z_des=None,
                           dt=0.1, robot_size=0.12, title="Animación del Robot"):
    """
    Anima un robot 2D dadas sus posiciones y orientaciones

    Args:
        y: array de forma (N,) con posiciones y
        z: array de forma (N,) con posiciones z
        phi: array de forma (N,) con ángulos phi
        y_des: array opcional de forma (N,) con posiciones y deseadas
        z_des: array opcional de forma (N,) con posiciones z deseadas
        dt: tiempo entre frames (para mostrar tiempo transcurrido)
        robot_size: tamaño del robot
        title: título de la animación

    Returns:
        HTML object para mostrar en Colab
    """

    # Crear figura
    fig, ax = plt.subplots(figsize=(10, 6))

    # Calcular límites automáticamente
    y_min, y_max = y.min() - 0.5, y.max() + 0.5
    z_min, z_max = z.min() - 0.5, z.max() + 0.5

    ax.set_xlim(y_min, y_max)
    ax.set_ylim(z_max, z_min)   # Invertir eje, debido a que Z es positivo hacia abajo
    ax.set_xlabel('Y (m)'); ax.set_ylabel('Z (m)')
    ax.set_title(title); ax.grid(True, alpha=0.3); ax.set_aspect('equal')

    # Elementos gráficos del robot
    robot_body, = ax.plot([], [], 'b-', linewidth=3, label='Robot')
    robot_prop1, = ax.plot([], [], 'ro', markersize=8)
    robot_prop2, = ax.plot([], [], 'ro', markersize=8)
    trajectory_line, = ax.plot([], [], 'b-', alpha=0.7, linewidth=2, label='Trayectoria')

    # Trayectoria objetivo si se proporciona
    if (y_des is not None) and (z_des is not None):
        target_line, = ax.plot(y_des, z_des, 'r--', alpha=0.5, label='Objetivo')
        current_target, = ax.plot([], [], 'r*', markersize=12)
        ax.legend()
    else:
        current_target = None
        ax.legend()

    # Arrays para la trayectoria seguida
    followed_y = []; followed_z = []

    def animate_frame(frame):
        if frame >= y.shape[0]:
            return robot_body, robot_prop1, robot_prop2, trajectory_line

        # Posición y orientación actual
        y_current = y[frame]; z_current = z[frame]; phi_current = phi[frame] 

        # Actualizar trayectoria seguida
        followed_y.append(y_current); followed_z.append(z_current)

        # Crear forma del robot
        body_points, prop1_pos, prop2_pos = create_robot_shape(y_current, z_current,
                                                              phi_current, size=robot_size)

        # Actualizar elementos gráficos del robot
        robot_body.set_data(body_points[:, 0].tolist() + [body_points[0, 0]],
                            body_points[:, 1].tolist() + [body_points[0, 1]])

        # Colocar en listas
        robot_prop1.set_data([prop1_pos[0]], [prop1_pos[1]])
        robot_prop2.set_data([prop2_pos[0]], [prop2_pos[1]])

        # Actualizar trayectoria seguida
        trajectory_line.set_data(followed_y, followed_z)

        # Actualizar objetivo si existe
        if current_target is not None and y_des is not None and z_des is not None:
            if frame < y_des.shape[0]:
                # Colocar coordenadas en listas
                current_target.set_data([y_des[frame]], [z_des[frame]])

        # Actualizar título con tiempo
        ax.set_title(f'{title} - t = {frame * dt:.1f}s')

        if current_target is not None:
            return robot_body, robot_prop1, robot_prop2, trajectory_line, current_target
        else:
            return robot_body, robot_prop1, robot_prop2, trajectory_line

    # Crear animación
    anim = animation.FuncAnimation(fig, animate_frame, frames=y.shape[0],
                                   interval=100, blit=True, repeat=True)

    # Cerrar figura estática
    plt.close()  

    return HTML(anim.to_jshtml())