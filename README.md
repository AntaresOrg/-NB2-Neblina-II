# -NB2-Neblina-II

Página do projeto do código do foguete de 3km Neblina II da equipe Antares de Foguetemodelismo.
Esta página ainda está em construção

## Estrutura Hierárquica do Projeto

```
Neblina II/
├── include/                        # Pasta raiz dos headers do projeto
│   ├── avionics/                       # Pasta raiz dos headers da aviônica
│   │   ├── bmp280.h                        # Header do Barômetro (BMP280)
│   │   ├── flash_memory.h                  # Header da Memória Flash (W25Q128)
│   │   └── mpu6050.h                       # Header do Acelerômetro (MPU6050)
│   ├── configs/                        # Pasta contendo os arquivos de configurações do projeto
│   │   ├── launch_settings.h               # Arquivo de configurações de lançamento
│   │   └── pins.h                          # Arquivo de definição de pinagem do projeto
│   ├── telemetry/                      # Pasta raiz dos headers da telemetria
│   │   ├── lora.h                          # Header do LoRa (E22-900M30S)
│   │   └── gps.h                           # Header do GPS (NEO-M9N)
│   └── utils/                          # Pasta raiz dos headers dos arquivos de utilidade
│       ├── logs.h                          # Header do arquivo de gerenciamento de logs
│       └── i2c.h                           # Header do arquivo de comunicação I2C
├── src/                            # Pasta raiz dos arquivos fonte do projeto
│   ├── avionics/                       # Pasta raiz do código fonte da aviônica
│   │   ├── bmp280.c                        # Código fonte do Barômetro (BMP280)
│   │   ├── flash_memory.c                  # Código fonte da Memória Flash (W25Q128)
│   │   └── mpu6050.c                       # Código fonte do Acelerômetro (MPU6050)
│   ├── telemetry/                      # Pasta raiz do código fonte da telemetria
│   │   ├── lora.c                          # Código fonte do LoRa (E22-900M30S)
│   │   └── gps.c                           # Código fonte do GPS (NEO-M9N)
│   ├── utils/                          # Pasta raiz do código fonte dos arquivos de utilidade
│   │   ├── logs.c                          # Código fonte do arquivo de gerenciamento de logs
│   │   └── i2c.c                           # Código fonte do arquivo de comunicação I2C
│   ├── CMakeLists.txt                  # Arquivo autogerado para compilação
│   ├── flight_routine.c                # Código fonte da rotina de voo do projeto
│   └── main.c                          # Código fonte do arquivo de entrada do código
├── .gitignore                      # Arquivo do git para lista de ignorados
├── CMakeLists.txt                  # Arquivo de linkagem do C
├── README.md                       # README da landing page do projeto
├── platformio.ini                  # Arquivo de configuração do platformio
└── sdkconfig                       # SDK autogerado pelo platformio
```
