#ifndef TEXTURE_MANAGER_H
#define TEXTURE_MANAGER_H

#ifdef MAC
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <stdio.h>

typedef enum { RAW_IMAGE, BMP_IMAGE, BMP_ALPHA_IMAGE } image_type_t;

// Image type - contains height, width, and data
typedef struct {
  unsigned long sizeX;
  unsigned long sizeY;
  unsigned char *data;
} image_t;

class TextureManager {
 public:
  int LoadTexture(GLuint *TID, char *filename, image_type_t type, bool wrap = true);
  int BuildColorTexture(GLuint *TID, unsigned char r, unsigned char g, unsigned char b);

 private:
  int LoadBMPImage(char *filename, FILE *file, image_t *image);
  int LoadBMPAlphaImage(char *filename, FILE *file, image_t *image);
  int LoadRAWImage(char *filename, FILE *file, unsigned int w, unsigned int h, image_t *image);
};

#endif
