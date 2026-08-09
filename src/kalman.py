import numpy as np

class KeepKalman:
    def __init__(self, mu0, P0, A, B, C, Q, R):
        """
        Parâmetros gerais para o filtro de Kalman. É um caso
        geral e poderá ser usado para a Parte B e depois no caso do IMU.

        Os parâmetros genéricos que devem ser fonecidos são:
        
        mu0, o vetor de estado inicial; P0, matriz da incerteza inicial da estimativa do estado;
        A, matriz de transição de estado; B matriz de controle; C; matriz de medição;
        Q, incerteza do modelo/predição; R, incerteza da medição.
        """
        self.mu = np.array(mu0, dtype=float)
        self.P = np.array(P0, dtype=float)
        self.A = np.array(A, dtype=float)
        self.B = np.array(B, dtype=float)
        self.C = np.array(C, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float) 
        
    def predicao(self, u):
        """
        Etapa de Predição:
        Nesta parte o estado passado do sistema é usado para projetar o estado e a incerteza no momento atual por meio
        de modelos (no caso do giroscópio, o dado do sensor entra no modelo físico, mas não é um cálculo direto de estado)
        """
        u = np.array(u, dtype=float)

        mu_pred = self.A @ self.mu + self.B @ u   # equação matricial para atualização do estado (sem correção)
        
        P_pred = self.A @ self.P @ self.A.T + self.Q   #  equação matricial de estimativa da incerteza da atualização

        self.mu = mu_pred
        self.P = P_pred

        return self.mu

    def correcao(self, z):
        """
        Etapa de Atualização (Correção):
        Nesta parte o filtro pega a medição real do sensor, calcula o grau de confiança nessa medição e ajusta a
        previsão original e a incerteza
        """
        z = np.array(z, dtype=float)

        K = self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R)  # Equação matricial do ganho de Kalman

        mu_new = self.mu + K @ (z - self.C @ self.mu)   # atualização do estado ponderado pelo filtro de Kalman

        P_new = self.P - K @ self.C @ self.P  # atualização da incerteza ponderada pelo filtro de Kalman

        self.mu = mu_new
        self.P = P_new

        return self.mu
