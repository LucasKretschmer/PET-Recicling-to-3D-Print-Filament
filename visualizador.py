import serial
import matplotlib.pyplot as plt
import time

PORTA_SERIAL = 'COM12'  # PORTA Serial Emulada por BT
BAUD_RATE = 115200

ser = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=1)
time.sleep(2) 

plt.ion()
fig, ax = plt.subplots()
trajeto_x = []
trajeto_y = []

linha_plot, = ax.plot([], [], color='blue', linewidth=2)
ax.set_title("Desenho no Plano Cartesiano - Trajetória do Carrinho")
ax.set_xlabel("X (cm)")
ax.set_ylabel("Y (cm)")
ax.grid(True)

# Ajusta limites iniciais do gráfico
ax.set_xlim(-1000, 1000)
ax.set_ylim(-1000, 1000)

try:
    while True:
        linha = ser.readline().decode().strip()

        if linha.startswith("x =") and "y =" in linha:
            try:
                # Remove partes desnecessárias e extrai os valores
                linha_limpa = linha.replace("x =", "").replace("y =", "").replace("theta =", "").replace("cm", "").replace("graus", "")
                partes = linha_limpa.split(",")

                x = float(partes[0].strip())
                y = float(partes[1].strip())
                # theta = float(partes[2].strip())  # Se quiser usar depois

                # Adiciona à lista
                trajeto_x.append(x)
                trajeto_y.append(y)

                # Atualiza o gráfico
                linha_plot.set_data(trajeto_x, trajeto_y)
                ax.relim()
                ax.autoscale_view()
                plt.pause(0.01)

            except ValueError:
                print("Erro ao converter linha:", linha)

except KeyboardInterrupt:
    print("Finalizado pelo usuário.")
    ser.close()
