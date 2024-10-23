import serial
import time

def find_arduino_port():
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if 'Arduino' in port.description:
                return port.device
    except ImportError:
        print("Error: No se pudo importar serial.tools.list_ports. Asegúrate de que pyserial esté instalado correctamente.")
    return None

def setup_serial(baud_rate=9600):
    port = find_arduino_port()
    if port is None:
        print("No se encontró un Arduino conectado.")
        return None
    arduino = serial.Serial(port, baud_rate)
    time.sleep(2)  # Espera a que el Arduino se inicialice
    return arduino

def control_leds(arduino, led_states):
    if arduino is not None:
        arduino.write(f"{led_states}\n".encode())

def main():
    arduino = setup_serial()
    if arduino is None:
        return  # Sale si no se encuentra un Arduino
    
    while True:
        # Enciende los LEDs (todos encendidos)
        led_states_on = 0b11111111  # Todos los LEDs encendidos
        control_leds(arduino, led_states_on)
        print("LEDs encendidos")
        time.sleep(1)  # Espera 1 segundo
        
        # Apaga los LEDs (todos apagados)
        led_states_off = 0b00000000  # Todos los LEDs apagados
        control_leds(arduino, led_states_off)
        print("LEDs apagados")
        time.sleep(1)  # Espera 1 segundo

if __name__ == "__main__":
    main()
