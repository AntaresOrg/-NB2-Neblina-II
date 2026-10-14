# Documentação Técnica do Código: Sistema de Aviônica EKF (ESP32)

Este documento detalha a implementação interna do código-fonte para o sistema de navegação e atitude embarcado em foguetes de alta performance. O software utiliza o framework **ESP-IDF** com **FreeRTOS** e executa a fusão de sensores de uma IMU **MPU6050** e um barômetro **BMP280** através de um **Filtro de Kalman Estendido Multiplicativo (Error-State EKF)** de $9$ estados.

---

## 🏗️ 1. Estrutura e Arquitetura de Estados (`ekf.h` / `ekf.c`)

O filtro de Kalman implementado adota a formulação de **Error-State (ES-EKF)**, onde o estado real é decomposto em um estado nominal e um vetor de erro infinitesimal de $N = 9$ dimensões:

$$\delta\mathbf{x} = \begin{bmatrix} \delta\boldsymbol{\theta}_{(3)} & \delta\mathbf{b}_{g(3)} & \delta z_{(1)} & \delta v_{z(1)} & \delta b_{a(1)} \end{bmatrix}^T$$

### Estruturas de Dados Principais
* **`ekf_params_t`**: Armazena as densidades de ruído de processo, desvios padrão de medição (`acc_sigma`, `baro_sigma`, `zupt_gyro_sigma`, `zupt_vel_sigma`), limiares de rejeição de aceleração (`acc_reject`, `acc_inflate`) e critérios de deteção de repouso no solo (`still_gyro`, `still_acc`, `still_samples`).
* **`ekf_t`**: Contém o estado nominal (`q[4]` quaternião de rotação *body-to-nav*, `bg[3]` viés do giroscópio, altitude $z$, velocidade vertical $v_z$, e viés do acelerômetro vertical $b_a$), além da matriz de covariância de erro $P_{9 \times 9}$ e contadores de repouso.

---

## 🔄 2. Funções Matemáticas e Ciclo de Execução do EKF

### A. Inicialização (`ekf_init`)
* Converte a gravidade média medida estaticamente no arranque para estimar os ângulos iniciais de *Roll* e *Pitch* utilizando trigonometria direta:
  $$\text{roll} = \text{atan2}(a_y, a_z), \quad \text{pitch} = \text{atan2}(-a_x, \sqrt{a_y^2 + a_z^2})$$
* Inicializa o quaternião nominal $\mathbf{q}$ e define a diagonal inicial da matriz de covariância $P$ (maior incerteza inicial em altitude e viés).

### B. Predição (`ekf_predict`)
Executada a cada ciclo de $10\,\text{ms}$ ($100\,\text{Hz}$):
1. Subtrai o viés atual dos dados brutos do giroscópio: $\mathbf{w} = \boldsymbol{\omega} - \mathbf{b}_g$.
2. Converte o quaternião nominal em matriz de rotação $R_{\text{body}\to\text{nav}}$ (`quat_to_rot`).
3. Calcula a aceleração vertical real no referencial de navegação (eixo $Z$ para cima):
   $$a_{\text{up}} = R_{30}a_x + R_{31}a_y + R_{32}a_z - g_{\text{std}} - b_a$$
4. Integra o estado cinemático nominal:
   $$z \leftarrow z + v_z \Delta t + \frac{1}{2}a_{\text{up}}\Delta t^2, \quad v_z \leftarrow v_z + a_{\text{up}}\Delta t$$
5. Atualiza o quaternião por rotação incremental usando o mapa exponencial (`quat_from_rotvec`).
6. Monta a matriz de transição de estado discreta $\Phi = I + F \cdot \Delta t$ considerando as derivadas parciais do erro de atitude e cinemática vertical.
7. Propaga a covariância: $P \leftarrow \Phi P \Phi^T + Q_d$, adicionando os ruídos de processo nas diagonais correspondentes.

### C. Atualizações de Correção (Atualização Escalar Sequencial)
O código implementa a função `scalar_update` para processar as medições de forma desacoplada e eficiente:
* **`ekf_update_accel`**: Verifica a norma da aceleração específica medida. Se o desvio em relação a $1\,\text{g}$ exceder `acc_reject`, a amostra é descartada para evitar corromper a atitude durante manobras extremas. Caso contrário, a covariância de medição $R_v$ sofre **inflação adaptativa** baseada no desvio ($R_v \times (1 + \text{inflate} \cdot \text{dev}^2)$) e corrige os eixos de atitude projetados.
* **`ekf_update_baro`**: Atualiza a altitude $z$ comparando-a diretamente com a leitura barométrica convertida, aplicando um teste de limiar Chi-quadrado ($gate = 9.0$).
* **`ekf_update_zupt`**: Monitora continuamente se o foguete está parado no solo. Se o giroscópio corrigido e a aceleração permanecerem abaixo dos limiares por um número consecutivo de amostras (`still_samples`), aplica correções de velocidade nula ($v_z = 0$) e ancora firmemente o viés do giroscópio (o que estabiliza inclusive o eixo de *Yaw* que não é observável apenas por gravidade).

---

## 🌡️ 3. Driver do Barômetro BMP280 (`bmp280.c`)

O código manipula diretamente os registos físicos do sensor via I2C:
* **Inicialização e Calibração:** Lê o bloco contínuo de $24$ bytes a partir do endereço base `REG_CALIB` (`0x88`). Reconstri os coeficientes de calibração de fábrica de $16$ bits (`T1..T3` e `P1..P9`) através de deslocamentos de bits bitwise (`<< 8 | c[i]`).
* **Compensação Matemática:** Utiliza as funções estáticas `comp_T` e `comp_P` baseadas estritamente na folha de dados (*datasheet*) da Bosch para computar a variável intermédia de temperatura `t_fine` e calcular a pressão final em formato de ponto fixo Q24.8.
* **Conversão Barométrica:** A função `bmp280_altitude` converte a pressão absoluta em altitude métrica usando a fórmula padrão barométrica da atmosfera ISA.

---

## 🧭 4. Driver da IMU MPU6050 (`mpu6050.c`)

* Configura os registos internos de gestão de energia (`REG_PWR_MGMT_1`), o divisor de taxa de amostragem (`REG_SMPLRT_DIV`) para fixar o ciclo em $100\,\text{Hz}$, e o filtro passa-baixas digital interno (DLPF em $44\,\text{Hz}$).
* Define as faixas de operação em $\pm 500\,^{\circ}/\text{s}$ para o giroscópio e $\pm 4\,\text{g}$ para o acelerômetro.
* A função `mpu6050_read` lê em rajada (*burst read*) $14$ bytes consecutivos a partir de `REG_ACCEL_XOUT_H`, convertendo os dados brutos de $16$ bits em unidades físicas padrão do SI: radianos por segundo ($\text{rad/s}$) para rotações e metros por segundo ao quadrado ($\text{m/s}^2$) para aceleração.

---

## 🧵 5. Tarefa Principal e Orquestração (`main.c`)

O ficheiro principal implementa o fluxo concorrente utilizando as APIs do FreeRTOS:
* **Calibração Estática Inicial:** Na função `ekf_task`, o sistema bloqueia a execução durante os primeiros $2$ segundos ($200$ amostras) para recolher as médias estáticas de giroscópio (`gmean`), acelerómetro (`amean`) e a pressão barométrica de referência ao nível da rampa de lançamento (`p0`).
* **Sincronismo Temporal:** Utiliza `vTaskDelayUntil` associado ao carimbo temporal de alta precisão (`esp_timer_get_time`) para calcular o $\Delta t$ exato entre iterações, mitigando instabilidades de temporização.
* **Divisão de Tarefas por Taxa:**
  * **$100\,\text{Hz}$:** Leitura da IMU, predição do EKF e atualizações de aceleração e ZUPT.
  * **$20\,\text{Hz}$ (`BARO_DIV = 5`):** Amostragem do sensor barométrico e execução da correção de altitude no EKF.
  * **$10\,\text{Hz}$ (`PRINT_DIV = 10`):** Extração de ângulos de Euler (`ekf_get_euler`) e envio de telemetria formatada via porta série (`printf`).
