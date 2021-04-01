# This is a sample Python script.

# Press ⌃R to execute it or replace it with your code.
# Press Double ⇧ to search everywhere for classes, files, tool windows, actions, and settings.

from math import *
import numpy as np
import sympy as sp  # librería para cálculo simbólico
import sim  # librería para conectar con CoppeliaSim
import time

def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press ⌘F8 to toggle the breakpoint.


def connect(port):
    # Establece la conexión a VREP
    # port debe coincidir con el puerto de conexión en VREP
    # retorna el número de cliente o -1 si no puede establecer conexión
    sim.simxFinish(-1)  # just in case, close all opened connections
    client_id = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)  # Conectarse
    if client_id == 0:
        print("conectado a", port)
    else:
        print("no se pudo conectar")
    return client_id


# Establece la posición de las articulaciones expresadas en radianes
def set_joints(q):
    ret_code = sim.simxSetJointTargetPosition(clientID, joint1, q[0], sim.simx_opmode_oneshot)
    ret_code = sim.simxSetJointTargetPosition(clientID, joint2, q[1], sim.simx_opmode_oneshot)
    ret_code = sim.simxSetJointTargetPosition(clientID, joint3, q[2], sim.simx_opmode_oneshot)


# Definimos una función para construir las matrices de transformación
# en forma simbóĺica a partir de los parámetros D-H
def sym_transform_dh(theta, d, a, alpha):
    # theta y alpha en radianes
    # d y a en metros
    Rz = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0, 0],
                    [sp.sin(theta), sp.cos(theta), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    tz = sp.Matrix([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, d],
                    [0, 0, 0, 1]])
    ta = sp.Matrix([[1, 0, 0, a],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    Rx = sp.Matrix([[1, 0, 0, 0],
                    [0, sp.cos(alpha), -sp.sin(alpha), 0],
                    [0, sp.sin(alpha), sp.cos(alpha), 0],
                    [0, 0, 0, 1]])
    T = Rz * tz * ta * Rx
    return T


# realiza la transformación de un punto especificado por (x,y,z) para
# que pueda ser operado con matrices de transformación
def transform_p(x, y, z):
    T = sp.Matrix([[1, 0, 0, x],
                   [0, 1, 0, y],
                   [0, 0, 1, z],
                   [0, 0, 0, 1]])
    return T


def connect_coppelia(port):
    # Requerimos los manejadores para las articulaciones y el Dummy
    clientID = connect(port)

    retCode, CoM = sim.simxGetObjectHandle(clientID, 'CoM', sim.simx_opmode_blocking)
    retCode, tip = sim.simxGetObjectHandle(clientID, 'tip', sim.simx_opmode_blocking)
    retCode, joint1 = sim.simxGetObjectHandle(clientID, 'MTB_joint1_1', sim.simx_opmode_blocking)
    retCode, joint2 = sim.simxGetObjectHandle(clientID, 'MTB_joint2_1', sim.simx_opmode_blocking)
    retCode, joint3 = sim.simxGetObjectHandle(clientID, 'MTB_joint3_1', sim.simx_opmode_blocking)
    return clientID, CoM, tip, joint1, joint2, joint3


def get_CoM_position_and_orientation(handle):
    [r, p] = sim.simxGetObjectPosition(clientID, handle, -1, sim.simx_opmode_blocking)
    [r, o] = sim.simxGetObjectOrientation(clientID, handle, -1, sim.simx_opmode_blocking)
    return [r, p, o]

# return the inverse of a transformation matrix (4x4)
def Inverse_Transformation(T):
    n = T[0:3, 0]
    o = T[0:3, 1]
    a = T[0:3, 2]
    p = T[0:3, 3]

    inv_t = sp.Matrix(n.T.col_insert(3, sp.Matrix(-n.T * p)))
    inv_t = inv_t.row_insert(inv_t.rows, sp.Matrix(o.T.col_insert(3, sp.Matrix(-o.T * p))))
    inv_t = inv_t.row_insert(inv_t.rows, sp.Matrix(a.T.col_insert(3, sp.Matrix(-a.T * p))))
    inv_t = inv_t.row_insert(inv_t.rows, sp.Matrix([[0, 0, 0, 1]]))

    return inv_t


# Realiza la cinemática directa del esquema CoppeliaSIM MTB_IK_v4_Florian_rotado_trasladado.ttt
# Recibe como parámetros los ángulos en radianes de las articulaciones
# Devuelve la posición del efector final
# alpha (a), beta (b), y gamma (g) son los ángulos de rotación de Dummy CoM
def forward_kinematics(P, O, R):
    l1 = 39.97e-3
    l2 = 22.5e-3
    l3 = 67.5e-3
    l4 = 90.5e-3

    # Orientación del CoM
    [a,b,c] = O

    # Ángulos a girar
    [q1,q2,q3] = R

    qz = sp.symbols('qz')
    rz = sp.Matrix([[sp.cos(qz), -sp.sin(qz), 0, 0],
                    [sp.sin(qz), sp.cos(qz), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Rotación en Y de pi/2 para pasar del frame 0 al frame 1 (hip)
    qy = sp.symbols('qy')
    ry = sp.Matrix([[sp.cos(qy), 0, sp.sin(qy), 0],
                    [0, 1, 0, 0],
                    [-sp.sin(qy), 0, sp.cos(qy), 0],
                    [0, 0, 0, 1]
                    ])

    qx = sp.symbols('qx')
    rx = sp.Matrix([[1, 0, 0, 0],
                    [0, sp.cos(qx), -sp.sin(qx), 0],
                    [0, sp.sin(qx), sp.cos(qx), 0],
                    [0, 0, 0, 1]])

    # transformación debido a valores de la IMU con rotaciones consecutivas
    A00 = rx.subs({qx: a}) * ry.subs({qy: b}) * rz.subs({qz: c})

    # Matriz de desplazamiento del CoM hacia la primera articulación de la pierna delantera derecha
    T0 = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])

    # Transformacion desde la orientación del CoM a la primera articulación
    A01 = rz.subs({qz: sp.pi / 2}) * ry.subs({qy: sp.pi / 2})

    # Rotación sobre la cadera, traslación sobre X (l2) y rotación sobre eje X de π / 2
    A12 = sym_transform_dh(q1, 0, l2, sp.pi / 2)

    # Traslación sobre X de -l1 hacia la articulación 2
    A23 = sym_transform_dh(0, -l1, 0, 0)

    # Rotación de q2 sobre la articulación 2 y desplazamiento en X de l3
    A34 = sym_transform_dh(q2, 0, l3, 0)

    # Rotación de q3 sobre la articulación 3 y desplazamiento en X de l4
    A45 = sym_transform_dh(q3, 0, l4, 0)

    # Punto inicial
    Pi = transform_p(P[0], P[1], P[2])

    # Transformación total desde el punto O
    T05 = sp.simplify(Pi * A00 * T0 * A01 * A12 * A23 * A34 * A45)

    x_tip = T05[0, 3]
    y_tip = T05[1, 3]
    z_tip = T05[2, 3]

    return x_tip, y_tip, z_tip


# Realiza la cinemática inversa del esquema CoppeliaSIM MTB_IK_v4_Florian_rotado_trasladado.ttt
# mediante aproximación numérica
# P: posición del CoM
# O: orientación del CoM
# Pf: Punto final a alcanzar
def inverse_kinematics_num(P, O, Pf):
    l1 = 39.97e-3
    l2 = 22.5e-3
    l3 = 67.5e-3
    l4 = 90.5e-3

    # Orientación del CoM
    [a, b, c] = O

    # Posición final
    [x, y, z] = Pf

    q1 = sp.symbols('q1')
    q2 = sp.symbols('q2')
    q3 = sp.symbols('q3')

    qz = sp.symbols('qz')
    rz = sp.Matrix([[sp.cos(qz), -sp.sin(qz), 0, 0],
                    [sp.sin(qz), sp.cos(qz), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Rotación en Y de pi/2 para pasar del frame 0 al frame 1 (hip)
    qy = sp.symbols('qy')
    ry = sp.Matrix([[sp.cos(qy), 0, sp.sin(qy), 0],
                    [0, 1, 0, 0],
                    [-sp.sin(qy), 0, sp.cos(qy), 0],
                    [0, 0, 0, 1]
                    ])

    qx = sp.symbols('qx')
    rx = sp.Matrix([[1, 0, 0, 0],
                    [0, sp.cos(qx), -sp.sin(qx), 0],
                    [0, sp.sin(qx), sp.cos(qx), 0],
                    [0, 0, 0, 1]])

    # transformación debido a valores de la IMU con rotaciones consecutivas
    A00 = rx.subs({qx: a}) * ry.subs({qy: b}) * rz.subs({qz: c})

    # Matriz de desplazamiento del CoM hacia la primera articulación de la pierna delantera derecha
    T0 = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])

    # Transformacion desde la orientación del CoM a la primera articulación
    A01 = rz.subs({qz: sp.pi / 2}) * ry.subs({qy: sp.pi / 2})

    # Rotación sobre la cadera, traslación sobre X (l2) y rotación sobre eje X de π / 2
    A12 = sym_transform_dh(q1, 0, l2, sp.pi / 2)

    # Traslación sobre X de -l1 hacia la articulación 2
    A23 = sym_transform_dh(0, -l1, 0, 0)

    # Rotación de q2 sobre la articulación 2 y desplazamiento en X de l3
    A34 = sym_transform_dh(q2, 0, l3, 0)

    # Rotación de q3 sobre la articulación 3 y desplazamiento en X de l4
    A45 = sym_transform_dh(q3, 0, l4, 0)

    # Punto inicial (CoM)
    Pi = transform_p(P[0], P[1], P[2])

    # Transformación total desde el punto O
    T05 = sp.simplify(Pi * A00 * T0 * A01 * A12 * A23 * A34 * A45)

    eq1 = T05[0, 3] - x
    eq2 = T05[1, 3] - y
    eq3 = T05[2, 3] - z

    q = sp.nsolve((eq1, eq2, eq3), (q1, q2, q3), (1, 1, 1))
    print('Degrees: ', degrees(q[0]), degrees(q[1]), degrees(q[2]))

    # Se devuelve como array numpy
    return np.array(q).astype(np.float64)



# Realiza la cinemática inversa del esquema CoppeliaSIM MTB_IK_v4_Florian_rotado_trasladado.ttt
# l1 = 25e-3, l2 = 20e-3, l3 = 80e-3, l4 = 80e-3
# mediante resolución geométrica
def inverse_kinematics_geo(p_CoM, o_CoM, Pf):
    # Se prueba IK Basic simulation by user Florian Wilk
    # https://gitlab.com/custom_robots/spotmicroai/simulation/-/tree/master/Basic%20simulation%20by%20user%20Florian%20Wilk/Kinematics
    # https://gitlab.com/custom_robots/spotmicroai/simulation/-/blob/master/Basic%20simulation%20by%20user%20Florian%20Wilk/Kinematics/Kinematic.ipynb

    l1 = 39.97e-3
    l2 = 22.5e-3
    l3 = 67.5e-3
    l4 = 90.5e-3

    # Orientación del CoM
    [qx, qy, qz] = o_CoM

    # Posición final
    [x, y, z] = Pf

    Pt = transform_p(x,y,z)

    # Giro de la posición original (*Florian_OK) hacia la nueva posición (*Florian_rotado)
    # respecto del frame del mundo: -90º sobre X, para poder especificar el punto destino
    # referenciado al frame del mundo
    Rx = sp.Matrix([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]])

    #[qx, qy, qz] = np.radians([30, 15, -35])
    # Giro de la figura en X, luego en Y, luego en Z, respecto del mundo!
    RoX = sp.Matrix([[1, 0, 0, 0], [0, cos(qx), -sin(qx), 0], [0, sin(qx), cos(qx), 0], [0, 0, 0, 1]])
    RoY = sp.Matrix([[cos(qy), 0, sin(qy), 0], [0, 1, 0, 0], [-sin(qy), 0, cos(qy), 0], [0, 0, 0, 1]])
    RoZ = sp.Matrix([[cos(qz), -sin(qz), 0, 0], [sin(qz), cos(qz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # Se rota la traslación según los ángulos de inclinación
    T_pata = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])

    # Traslación compuesta: Orientación X,Y,Z
    T = RoX * RoY * RoZ * T_pata

    # Orientación de la figura según IMU: Rotación combinada (X,Y,Z) respecto a sí mismo
    Pi = Rx.inv() * RoZ.inv() * RoY.inv() * RoX.inv() * (Pt - T)

    [x, y, z] = np.float64([Pi[0, 3], Pi[1, 3], Pi[2, 3]])

    print([x, y, z])

    F = sqrt(x ** 2 + y ** 2 - l1 ** 2)
    G = F - l2
    H = sqrt(G ** 2 + z ** 2)

    theta1 = atan2(y, x) - atan2(F, -l1)

    D = (H ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4)
    # Codo abajo 
    theta3 = -acos(D)
    # Codo arriba
    theta3 = +acos(D)

    theta2 = atan2(z, G) - atan2(l4 * sin(theta3), l3 + l4 * cos(theta3))

    print('Degrees: ', np.degrees([theta1, theta2, theta3]))
    return theta1, theta2, theta3


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')



# Requerimos los manejadores para las articulaciones y el Dummy
clientID, CoM, tip, joint1, joint2, joint3 = connect_coppelia(19999)

r, p, o = get_CoM_position_and_orientation(CoM)

# Posicionamos en valores iniciales
#set_joints([0, 0, 0])
#time.sleep(1)

set_joints([radians(45),radians(45),radians(45)])
# Altura desde el CoM al suelo
H = p[2]

[x, y, z] = forward_kinematics(p, o, np.deg2rad([45,45,45]))

# z + H muestra la altura respecto al suelo si posición del CoM se establace como [0,0,0]
# en otro caso, z es la altura sobre el suelo, ya que la posición inicial ya tiene la altura
print([x, y, z])

# z - H, siendo z la altura sobre el suelo (+) o bajo el suelo (-)
# cuando p tiene altura se pone z directamente

#q = inverse_kinematics_num(p, o, [x, y, z - 0.01])
#set_joints(q)
#time.sleep(1)

set_joints([radians(45),radians(45),radians(45)])
time.sleep(1)

q = inverse_kinematics_geo(p,o, [-21e-3, -19e-3, -H + 0.26])
set_joints(q)
print (H)





