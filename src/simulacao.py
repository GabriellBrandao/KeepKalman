import matplotlib.pyplot as plt

# Nesta parte, o vetor de estado é escalar porque não foi considerado o bias do giroscópio para este caso (apenas ângulo)
# Assim as matrizes P, A, B, C, Q, R, P, P0 e mu0 serão escalares (versão escalar do slide)

np.random.seed(40) # adicionado dar o mesmo resultado

# Dados necessários para a simulação
var_gir = 0.05   # Giroscópio (Baixa variância)
var_acel = 0.5   # Acelerômetro (Alta variância)
dt = 0.01 # intervalo de tempo para atualização do estado

# Criação dos sinais reais
tempo = np.arange(0, 10, dt)
theta_real = np.sin(tempo)
omega_real = np.cos(tempo)

# Adicionando os ruidos para os sinais reais
gir_medido = omega_real + np.random.normal(0, np.sqrt(var_gir), len(tempo))
acel_medido = theta_real + np.random.normal(0, np.sqrt(var_acel), len(tempo))

# Cálculo das incertezas
Q = [[var_gir * (dt**2)]] # incerteza do giroscópio
R = [[var_acel]]  # incerteza do acelerômetro

# Parametros para inserir na classe
# mu0=theta_acc0, P0 = R
mu0 = [[acel_medido[0]]]; P0 = R; A = [[1]]; B = [[dt]]; C = [[1]]

filtro = KeepKalman(mu0, P0, A, B, C, Q, R)

theta_filtrado = []
for i in range(len(tempo)):
    u = [[gir_medido[i]]]
    z = [[acel_medido[i]]]
    filtro.predicao(u)
    estado_atual = filtro.correcao(z)
    
    theta_filtrado.append(estado_atual[0, 0])

theta_filtrado = np.array(theta_filtrado)

# Cálculo dos erros
eqm_acel = np.mean((theta_real - acel_medido)**2)
eqm_kalman = np.mean((theta_real - theta_filtrado)**2)
G = (1 - eqm_kalman/eqm_acel)*100

print(f"ERROS QUADRÁTICOS MÉDIOS (EQM) \nEQM Acel (sem o filtro): {eqm_acel:.4f}\nEQM Kalman: {eqm_kalman:.4f}\nGanho Estatisico: {G:.2f}%\n")

# Plots
plt.figure(figsize=(12, 6))
plt.plot(tempo, theta_real, 'r-', linewidth=2, label='Real')
plt.plot(tempo, acel_medido, 'k.', alpha=0.6, label='Medido')
plt.plot(tempo, theta_filtrado, 'b-', linewidth=2, label='Filtrado')
plt.xlabel('Tempo (s)'); plt.ylabel('Roll $\\theta$ (rad)')
plt.legend(); plt.grid(True, linestyle='--', alpha=0.6)
plt.tight_layout(); plt.show()
