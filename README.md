<div align="center">

# Proyecto Final de Sistemas de Control MR2002B (Equipo 1): PID Digital con Arduino y MATLAB


![Arduino](https://img.shields.io/badge/-Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)
![MATLAB](https://img.shields.io/badge/MATLAB-red?style=for-the-badge&logo=mathworks
)
</div>




Este repositorio contiene los archivos del proyecto final de la materia de **Sistemas de Control**. El objetivo principal es implementar un controlador PID discretizado para controlar un motor DC en lazo cerrado utilizando un **Arduino UNO** y un encoder magnético para retroalimentación. En este proyecto, se recopilan datos del sistema físico y, posteriormente, se utilizan herramientas de **MATLAB** para identificar la planta y ajustar las ganancias del controlador.

## Contenido del Repositorio

### 1. **Códigos de Arduino**
En la carpeta `Arduino` encontrarás los siguientes archivos:
- `PRBS_Generator.ino`: Código para generar una señal **PRBS** (Pseudo Random Binary Sequence) en el Arduino UNO, utilizada para recopilar datos del sistema.
- `PID_Controller.ino`: Código del controlador **PID discretizado** con las ganancias ajustadas, utilizado para controlar el motor DC.

### 2. **Archivos de MATLAB**
En la carpeta `MATLAB` se incluyen los scripts y archivos necesarios para la identificación de la planta y ajuste de las ganancias PID:
- `data_processing.m`: Script que procesa los datos recopilados por el Arduino y los guarda en un archivo CSV.
- `system_identification.m`: Script que utiliza los datos experimentales para generar una **función de transferencia** mediante la herramienta de identificación de sistemas de MATLAB.
- `pid_tuner.m`: Script que utiliza el **PID Tuner** de MATLAB para obtener los valores óptimos de las ganancias $K_p$, $K_i$, y $K_d$.

## Descripción del Proyecto

### 1. **Implementación del PID**
Este proyecto surge como una continuación de experimentos previos realizados en la clase, donde se utilizó un controlador **PID analógico**. En esta ocasión, el enfoque fue implementar un PID digital para un mayor control y flexibilidad. El controlador recibe como entrada la retroalimentación del encoder magnético y ajusta la salida para controlar la posición del motor DC.

### 2. **Identificación de la Planta**
Para obtener una representación precisa del sistema, se generó una señal PRBS para excitar la planta y registrar su respuesta. Estos datos se procesaron en **MATLAB**, y utilizando la herramienta de identificación de sistemas, se obtuvo una función de transferencia de segundo orden con una fiabilidad del 99.35\%.

### 3. **Ajuste del PID**
Con la función de transferencia identificada, se ajustaron las ganancias del controlador en **MATLAB** mediante el **PID Tuner**. Las ganancias finales $K_p$, $K_i$, y $K_d$ se implementaron en el código del Arduino para controlar el sistema en tiempo real.

## Requisitos

- **Arduino UNO** o cualquier otro microcontrolador compatible.
- **MATLAB** con la Toolbox de identificación de sistemas y control PID.
- **Arduino IDE** para compilar y subir el código al microcontrolador.
- **Osciloscopio** o **Serial Plotter** del Arduino IDE para visualizar las señales.

## Instrucciones de Uso

1. Clona este repositorio en tu máquina local:
   ```bash
   git clone https://github.com/HumbertoBM2/Equipo-1_ProyectoFinal_MR2002B.402.git
