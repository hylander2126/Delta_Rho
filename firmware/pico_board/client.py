import pygame, requests
import numpy as np

pygame.init()

display_width = 1280
display_height = 720

"""screen = pygame.display.set_mode((display_width, display_height))
pygame.display.set_caption('Window')"""

running = True

gain = 80
jacobian = np.array([[1.7321,  1.0000, -0.5792],
            [0,       -2.0000, 2.7400],
            [-1.7321, 1.0000, -0.5792]]).reshape(3, 3)

motionvector = np.array([0, 0, 0]).reshape(3, 1) #x, y, r

url = 'http://192.168.0.15/'

requests.post(url, "turn")

"""while running:
    for event in pygame.event.get():
        pass
    mouse = pygame.mouse.get_pos()
    deltax = display_width/2-mouse[0]
    deltay = display_height/2-mouse[1]
    motionvector = np.array([deltax, deltay, 0])
    output = motionvector.dot(jacobian)
    print(output)
    #send requests to run motors"""