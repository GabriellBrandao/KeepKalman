# AVALIAÇÃO DO FILTRO DE KALMAN NA FUSÃO DE SENSORES (CASO 1D)
Neste repositório, há scripts desenvolvidos para avaliar o filtro de Kalman aplicado na fusão de sensores, mais especificamente, na estimativa do ângulo roll (caso 1D) por meio de giroscópio e acelerômetro. A ideia é entender o fundamento teórico que advém do Teorema de Bayes na construção do filtro. O filtro se baseia na adoção de distribuições Gaussianas para a incerteza do modelo dinâmico e dos sensores. Considerando ainda o comportamento linear dos sistemas, chega-se a um algoritmo recursivo que consegue estimar o estado do sistema com atualizações provenientes da medição dos sensores.

##  Integrantes:
* Gabriel Brandão Santos - 202500249

## Instruções de Instalação e Execução (Parte A e B):
### 1. Clonar o Repositório
```bash
git clone [https://github.com/GabriellBrandao/KeepKalman.git](https://github.com/GabriellBrandao/KeepKalman.git)
cd KeepKalman
```
### 2. Instalar as Dependências

```bash
pip install -r requirements.txt
```
### 3. Executar a Simulação (Parte B)

```bash
python src/simulacao.py
```

### 4. Executar a Implementação Prática (Parte C)
```bash
python src/main_pratica.py
```

## Guia de Execução da Simulação da IMU (Parte C)
Para a simulação da IMU, foi utlizado o Webots R2025a.
- Extraía o arquivo zipado chamado IMUSimulada.rar
- Para executar a simulação, inicie o simulador Webots no seu computador.
- No menu superior do Webots, clique em File e depois em Open World.
- Procure e abra a IMUSimulada que foi extraída.
- Entre na pasta IMUSimulada, depois abra a subpasta chamada worlds.
- Selecione o arquivo worlddekalman.wbt e clique em Abrir.
- Clique no botão de Play no topo da tela para iniciar a simulação.
