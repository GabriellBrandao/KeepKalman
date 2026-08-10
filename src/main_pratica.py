from controller import Robot
import numpy as np
import math
import matplotlib.pyplot as plt

# Add a classe que criei na Parte A para ser chamada com os dados obtidos aqui
# no Webot. As matrizes Q e R serão obtidas por meio da calibração dos sensores
# com o motor parado logo abaixo.
class KeepKalman:
    def __init__(self, mu0, P0, A, B, C, Q, R):
        self.mu = np.array(mu0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.A = np.array(A, dtype=float)
        self.B = np.array(B, dtype=float)
        self.C = np.array(C, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float) 
        
    def predicao(self, u):
        u = np.array(u, dtype=float)
        mu_pred = self.A @ self.mu + self.B @ u   
        P_pred = self.A @ self.P @ self.A.T + self.Q   
        self.mu = mu_pred
        self.P = P_pred
        return self.mu

    def correcao(self, z):
        z = np.array(z, dtype=float)
        K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R) 
        mu_new = self.mu + K @ (z - self.C @ self.mu)   
        P_new = self.P - K @ self.C @ self.P  
        self.mu = mu_new
        self.P = P_new
        return self.mu

# Código principal do Webots
def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0  # intervalo de tempo das amostras

    # chamando os sensores (acelerometro e o giroscopio)
    accel = robot.getDevice("accel")
    accel.enable(timestep)
    
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    
    motor = robot.getDevice("motor")
    
    # Parte para sintonizar as matrizes Q e R, o motor fica parado no começo para
    # que seja calculada a variância do acelerometro e do giroscopio. Forma que
    # estudei para tentar a sintonização das amostras de fomra mais objetiva
    motor.setPosition(0.0)
    motor.setVelocity(0.0)

    # eliminando os 10 primeiros passos para estabilização
    for _ in range(10):
        robot.step(timestep)

    # Parte para sintonizar as matrizes Q e R
    amostras_acc = []
    amostras_gyro = []

    # Coleta de 500 amostras com a IMU parada para depois ser calculada a variância
    for _ in range(500):
        if robot.step(timestep) == -1:
            return
        
        # Coleta do giroscópio
        wx, wy, wz = gyro.getValues()
        amostras_gyro.append(wx)
        
        # Coleta do acelerômetro e calcula do Roll
        ax, ay, az = accel.getValues()
        roll_acc = math.atan2(ay, az)
        amostras_acc.append(roll_acc)

    # Câlculo das variâncias com a IMU parada (no caso do giroscopio, será apenas
    # do ângulo, o bias é inputado abaixo).
    
    var_acc = np.var(amostras_acc)
    var_gyro = np.var(amostras_gyro)

    print("-" * 50)
    print("Sintonização Concluída:")
    print(f"Variância do Acelerômetro (Matriz R): {var_acc:.6f}")
    print(f"Variância do Giroscópio (Q_angulo): {var_gyro:.6f}")
    print("-" * 50)
    
    # Os valores da sintonização com IMU parada para as matrizes Q e R são add
    # O bias é assumido este valor por variar lentamente
    Q = [[var_gyro, 0.0], 
         [0.0, 0.003]]
                  
    R = [[var_acc]]

    # Definição das matrizes para o filtro e estados iniciais
    mu0 = [[0.0], [0.0]]
    P0 = [[var_acc, 0.0], [0.0, var_acc]]
    A = [[1.0, -dt], [0.0, 1.0]]
    B = [[dt], [0.0]]
    C = [[1.0, 0.0]]
    
    # Aqui minha classe é chamada com os parâmtros já definidos
    kf = KeepKalman(mu0, P0, A, B, C, Q, R)

    # plots
    plt.ion()
    fig, ax = plt.subplots(figsize=(9, 5))
    
    linha_acc, = ax.plot([], [], label='Medido (Acelerômetro)', color='red', alpha=0.5)
    linha_kf, = ax.plot([], [], label='Filtrado (Filtro de Kalman)', color='blue', linewidth=2.5)
    
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("Roll (Graus)")
    ax.legend(loc="upper right")
    ax.grid(True)
    
    tempo_hist = []
    acc_hist = []
    kf_hist = []
    janela_tempo = 5.0
    step_count = 0

    # Loop da simulação no Webots
    # O motor do webots é ligado aqui para iniciar a simulação
    motor.setPosition(float('inf'))
    motor.setVelocity(2.0)

    while robot.step(timestep) != -1:
        step_count += 1
        tempo_atual = robot.getTime()
        
        # Predição (Giroscópio)
        wx, wy, wz = gyro.getValues()
        u = [[wx]]  
        kf.predicao(u)

        # Correção (Acelerômetro)
        ax_val, ay_val, az_val = accel.getValues()
        roll_acc = math.atan2(ay_val, az_val)
        z = [[roll_acc]]
        mu_est = kf.correcao(z)

        # Atualização dos plots
        graus_acc = math.degrees(roll_acc)
        graus_kf = math.degrees(mu_est[0][0])

        tempo_hist.append(tempo_atual)
        acc_hist.append(graus_acc)
        kf_hist.append(graus_kf)

        if tempo_atual - tempo_hist[0] > janela_tempo:
            tempo_hist.pop(0)
            acc_hist.pop(0)
            kf_hist.pop(0)

        if step_count % 10 == 0:
            linha_acc.set_data(tempo_hist, acc_hist)
            linha_kf.set_data(tempo_hist, kf_hist)
            
            ax.set_xlim(tempo_hist[0], tempo_hist[-1] if len(tempo_hist) > 1 else tempo_hist[0] + 1)
            
            min_y = min(acc_hist) - 15
            max_y = max(acc_hist) + 15
            ax.set_ylim(min_y, max_y)
            
            fig.canvas.draw()
            fig.canvas.flush_events()

if __name__ == "__main__":
    main()
