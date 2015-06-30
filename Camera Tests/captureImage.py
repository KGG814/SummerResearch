#!/usr/bin/env python

def capture(cam = 0, fileName = "photo.bmp"):
  import pygame.camera
  pygame.camera.init()
  cam = pygame.camera.Camera(pygame.camera.list_cameras()[cam])
  cam.start()
  img = cam.get_image()
  import pygame.image
  pygame.image.save(img, fileName)
  pygame.camera.quit()

if __name__ == "__main__":
  capture()
