from machine import Pin
from car import CarControl

car = CarControl()

try:
    while True:
        car.Motor_forward()
except KeyboardInterrupt:
    car.finish()