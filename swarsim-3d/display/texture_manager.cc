#include "texture_manager.h"

#include <stdio.h>
#include <stdlib.h>

#ifndef ABS
#define ABS(x) (((x) < 0)?-(x):(x))
#endif

#define glError() {                \
    GLenum err = glGetError();                                          \
    while (err != GL_NO_ERROR) {                                        \
      fprintf(stderr, "glError: %s caught at %s:%u\n", (char *)gluErrorString(err), __FILE__, __LINE__); \
      err = glGetError();                                               \
    }                                                                   \
  }

int TextureManager::BuildColorTexture(GLuint *TID, unsigned char r, unsigned char g, unsigned char b) {
  unsigned char data[12];	// A 2x2 texture at 24 bits

  // Store the data,
  for(int i = 0; i < 12; i += 3) {
    data[i] = r;
    data[i+1] = g;
    data[i+2] = b;
  }

  // Generate the OpenGL texture id,
  glGenTextures(1, TID);

  // Bind this texture to its id,
  glBindTexture(GL_TEXTURE_2D, *TID);

  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

  // Use mipmapping filter,
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_NEAREST);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);

  // Generate the texture
  gluBuild2DMipmaps(GL_TEXTURE_2D, 3, 2, 2, GL_RGB, GL_UNSIGNED_BYTE, data);

  return 0;
}

int TextureManager::LoadTexture(GLuint *TID, char *filename, image_type_t type, bool wrap) {
  FILE *file;
  image_t image;
  int result = -1;

  if (!TID) return -1;

  // Open texture data.
  file = fopen(filename, "rb");
  if (file == NULL) return -1;
  switch (type) {
  case RAW_IMAGE:
    result = LoadRAWImage(filename, file, 256, 256, &image);
    break;
  case BMP_IMAGE:
    result = LoadBMPImage(filename, file, &image);
    break;
  case BMP_ALPHA_IMAGE:
    result = LoadBMPAlphaImage(filename, file, &image);
  }
  fclose(file);
  if (result) return -1;

  // allocate a texture name.
  glGenTextures(1, TID);

  // Select our current texture.
  glBindTexture(GL_TEXTURE_2D, *TID);

  glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap ? GL_REPEAT : GL_CLAMP);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap ? GL_REPEAT : GL_CLAMP);

  if (type == BMP_ALPHA_IMAGE)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image.sizeX, image.sizeY, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data);
  else
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.sizeX, image.sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data);

  // Free buffer.
  free(image.data);

  return 0;
}

int TextureManager::LoadBMPImage(char *filename, FILE *file, image_t *image) {
  unsigned long size;                 // size of the image in bytes.
  unsigned long i;                    // standard counter.
  unsigned short int planes;          // number of planes in image (must be 1).
  unsigned short int bpp;             // number of bits per pixel (must be 24).
  char temp;                          // temporary color storage for bgr-rgb conversion.

  // Seek through the bmp header, up to the width/height.
  fseek(file, 18, SEEK_CUR);

  // Read the width
  if ((i = fread(&(image->sizeX), 4, 1, file)) != 1) {
    printf("Error reading width from %s.\n", filename);
    return -1;
  }
  image->sizeX = (unsigned long)(ABS((signed short)((unsigned short)image->sizeX & 0xffff)));
  printf("Width of %s: %lu (x%lx)\n", filename, image->sizeX, image->sizeX);

  // Read the height
  if ((i = fread(&(image->sizeY), 4, 1, file)) != 1) {
    printf("Error reading height from %s.\n", filename);
    return -1;
  }
  image->sizeY = (unsigned long)(ABS((signed short)((unsigned short)image->sizeY & 0xffff)));
  printf("Height of %s: %lu (x%lx)\n", filename, image->sizeY, image->sizeY);

  // calculate the size (assuming 24 bits or 3 bytes per pixel).
  size = image->sizeX * image->sizeY * 3;

  // Read the planes.
  if ((fread(&planes, 2, 1, file)) != 1) {
    printf("Error reading planes from %s.\n", filename);
    return -1;
  }
  if (planes != 1) {
    printf("Planes from %s is not 1: %u\n", filename, planes);
    return -1;
  }

  // Read the bpp.
  if ((i = fread(&bpp, 2, 1, file)) != 1) {
    printf("Error reading bpp from %s.\n", filename);
    return -1;
  }
  if (bpp != 24) {
    printf("Bpp from %s is not 24: %u\n", filename, bpp);
    return -1;
  }

  // Seek past the rest of the bitmap header.
  fseek(file, 24, SEEK_CUR);

  // Read the data.
  image->data = (unsigned char *) malloc(size);
  if (image->data == NULL) {
    printf("Error allocating memory for color-corrected image data\n");
    return -1;
  }

  if ((i = fread(image->data, size, 1, file)) != 1) {
    printf("Error reading image data from %s.\n", filename);
    free(image->data);
    return -1;
  }

  for (i=0; i<size; i+=3) { // Reverse all of the colors. (bgr -> rgb).
    temp = image->data[i];
    image->data[i] = image->data[i+2];
    image->data[i+2] = temp;
  }

  return 0;
}

int TextureManager::LoadBMPAlphaImage(char *filename, FILE *file, image_t *image) {
  image_t orig;
  unsigned long i, j;
  unsigned long size;

  LoadBMPImage(filename, file, &orig);

  image->sizeX = orig.sizeX;
  image->sizeY = orig.sizeY;
  image->data = (unsigned char *)malloc(orig.sizeX * orig.sizeY * 4);
  size = orig.sizeX * orig.sizeY * 3;
  for (i = 0, j = 0; i < size; i+=3, j += 4) {
    image->data[j] = orig.data[i];
    image->data[j+1] = orig.data[i+1];
    image->data[j+2] = orig.data[i+2];
    image->data[j+3] = 255;
    if (orig.data[i] == 0 && orig.data[i+1] == 0 && orig.data[i+2] == 0)
      image->data[j+3] = 0;
  }

  free(orig.data);
  return 0;
}

int TextureManager::LoadRAWImage(char *filename, FILE *file, unsigned int w, unsigned int h, image_t *image) {
  if (!file || !image) return -1;

  // Allocate buffer.
  image->sizeX = 256;
  image->sizeY = 256;
  image->data = (unsigned char *)malloc(w*h*3);
  if (!image->data) {
    printf("Error allocating memory for color-corrected image data\n");
    return -1;
  }

  // Read texture data.
  unsigned int r = fread(image->data, sizeof(char), w*h*3, file);
  if (r != (unsigned int)(w*h*3)) {
    printf("Error while loading RAW texture from %s\n", filename);
    free(image->data);
    return -1;
  }

  return 0;
}
