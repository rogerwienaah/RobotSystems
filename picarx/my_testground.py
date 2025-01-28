from picarx_improved import Picarx
from time import sleep

px = Picarx()

while True:
    grayscale_data = px.get_grayscale_data()
    print(grayscale_data)
