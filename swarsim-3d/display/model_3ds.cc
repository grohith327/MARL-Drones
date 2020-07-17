#include "model_3ds.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

// The chunk's id numbers
#define MAIN3DS				0x4D4D
#define MAIN_VERS			0x0002
#define EDIT3DS			0x3D3D
#define MESH_VERS			0x3D3E
#define OBJECT			0x4000
#define TRIG_MESH		0x4100
#define VERT_LIST		0x4110
#define FACE_DESC		0x4120
#define FACE_MAT		0x4130
#define TEX_VERTS		0x4140
#define SMOOTH_GROUP	0x4150
#define LOCAL_COORDS	0x4160
#define MATERIAL			0xAFFF
#define MAT_NAME			0xA000
#define MAT_AMBIENT		0xA010
#define MAT_DIFFUSE		0xA020
#define MAT_SPECULAR		0xA030
#define SHINY_PERC		0xA040
#define SHINY_STR_PERC	0xA041
#define TRANS_PERC		0xA050
#define TRANS_FOFF_PERC	0xA052
#define REF_BLUR_PERC	0xA053
#define RENDER_TYPE		0xA100
#define SELF_ILLUM		0xA084
#define MAT_SELF_ILPCT	0xA08A
#define WIRE_THICKNESS	0xA087
#define MAT_TEXMAP		0xA200
#define MAT_MAPNAME		0xA300
#define ONE_UNIT			0x0100
#define KEYF3DS			0xB000
#define FRAMES			0xB008
#define MESH_INFO			0xB002
#define HIER_POS			0xB030
#define HIER_FATHER		0xB010
#define PIVOT_PT			0xB013
#define TRACK00			0xB020
#define TRACK01			0xB021
#define TRACK02			0xB022
#define	COLOR_RGB			0x0010
#define COLOR_TRU			0x0011
#define COLOR_TRUG			0x0012
#define COLOR_RGBG			0x0013
#define PERC_INT			0x0030
#define PERC_FLOAT			0x0031

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Model3DS::Model3DS()
{
  // Initialization

  // Don't show the normals by default
  shownormals = false;

  // The model is lit by default
  lit = true;

  // The model is visible by default
  visible = true;

  // Set up the default position
  pos.x = 0.0f;
  pos.y = 0.0f;
  pos.z = 0.0f;
  // Set up the default rotation
  rot.x = 0.0f;
  rot.y = 0.0f;
  rot.z = 0.0f;

  // Set up the path
  path = new char[80];
  path[0] = '\0';

  // Zero out our counters for MFC
  numObjects = 0;
  numMaterials = 0;

  // Set the scale to one
  scale = 1.0f;
}

Model3DS::~Model3DS()
{
  // We need to delete all textures
  for (int i = 0; i < numMaterials; i++) {
    if (!Materials[i].textured) continue;

    if (Materials[i].tex > 0) {
      glDeleteTextures(1, &Materials[i].tex);
      Materials[i].tex = 0;
    }
  }
}

void Model3DS::Load(const char *const_name)
{
  int nbytes = strlen(const_name) + 1;
  char *name = (char *)malloc(nbytes);
  memcpy(name, const_name, nbytes);

  // holds the main chunk header
  ChunkHeader main;

  // strip "'s
  if (strstr(name, "\""))
    name = strtok(name, "\"");

  // Find the path
  if (strstr(name, "/") || strstr(name, "\\")) {
    // Holds the name of the model minus the path
    char *temp;

    // Find the name without the path
    if (strstr(name, "/"))
      temp = strrchr(name, '/');
    else
      temp = strrchr(name, '\\');

    // Allocate space for the path
    path = new char[strlen(name)-strlen(temp)+1];

    // Get a pointer to the end of the path and name
    char *src = name + strlen(name) - 1;

    // Back up until a \ or the start
    while (src != path && !((*(src-1)) == '\\' || (*(src-1)) == '/'))
      src--;

    // Copy the path into path
    memcpy (path, name, src-name);
    path[src-name-1] = 0;
  }

  // Load the file
  printf("Loading 3DS model %s with textures: ", name);
  bin3ds = fopen(name,"rb");
  if (!bin3ds) {
    visible = false;
    fprintf(stderr, "The 3DS model '%s' could not be found!\n", name);
    return;
  }

  // Make sure we are at the beginning
  fseek(bin3ds, 0, SEEK_SET);

  // Load the Main Chunk's header
  fread(&main.id,sizeof(main.id),1,bin3ds);
  fread(&main.len,sizeof(main.len),1,bin3ds);

  // Start Processing
  MainChunkProcessor(main.len, ftell(bin3ds));

  // Don't need the file anymore so close it
  fclose(bin3ds);

  // Calculate the vertex normals
  CalculateNormals();

  // For future reference
  modelname = name;

  // Find the total number of faces and vertices
  totalFaces = 0;
  totalVerts = 0;

  for (int i = 0; i < numObjects; i ++)
    {
      totalFaces += Objects[i].numFaces/3;
      totalVerts += Objects[i].numVerts;
    }

  // If the object doesn't have any texcoords generate some
  for (int k = 0; k < numObjects; k++)
    {
      if (Objects[k].numTexCoords == 0)
        {
          // Set the number of texture coords
          Objects[k].numTexCoords = Objects[k].numVerts;

          // Allocate an array to hold the texture coordinates
          Objects[k].TexCoords = new GLfloat[Objects[k].numTexCoords * 2];

          // Make some texture coords
          for (int m = 0; m < Objects[k].numTexCoords; m++)
            {
              Objects[k].TexCoords[2*m] = Objects[k].Vertexes[3*m];
              Objects[k].TexCoords[2*m+1] = Objects[k].Vertexes[3*m+1];
            }
        }
    }

  // Let's build simple colored textures for the materials w/o a texture
  for (int j = 0; j < numMaterials; j++)
    {
      if (Materials[j].textured == false)
        {
          unsigned char r = Materials[j].color.r;
          unsigned char g = Materials[j].color.g;
          unsigned char b = Materials[j].color.b;
          textureManager.BuildColorTexture(&Materials[j].tex, r, g, b);
          printf("%d ", Materials[j].tex);
          Materials[j].textured = true;
        }
    }

  printf("\n");
}

void Model3DS::Draw()
{
  if (visible)
    {
      glPushAttrib(GL_LIGHTING_BIT);
      glPushMatrix();

      // Move the model
      glTranslatef(pos.x, pos.y, pos.z);

      // Rotate the model
      glRotatef(rot.x, 1.0f, 0.0f, 0.0f);
      glRotatef(rot.y, 0.0f, 1.0f, 0.0f);
      glRotatef(rot.z, 0.0f, 0.0f, 1.0f);

      glScalef(scale, scale, scale);

      // Loop through the objects
      for (int i = 0; i < numObjects; i++)
        {
          // Enable texture coordiantes, normals, and vertices arrays
          if (Objects[i].textured) {
            glEnableClientState(GL_TEXTURE_COORD_ARRAY);
          }
          if (lit)
            glEnableClientState(GL_NORMAL_ARRAY);
          glEnableClientState(GL_VERTEX_ARRAY);

          // Point them to the objects arrays
          if (Objects[i].textured)
            glTexCoordPointer(2, GL_FLOAT, 0, Objects[i].TexCoords);
          if (lit)
            glNormalPointer(GL_FLOAT, 0, Objects[i].Normals);
          glVertexPointer(3, GL_FLOAT, 0, Objects[i].Vertexes);

          // Loop through the faces as sorted by material and draw them
          for (int j = 0; j < Objects[i].numMatFaces; j ++)
            {
              if (Materials[Objects[i].MatFaces[j].MatIndex].textured) {
                // Use the material's texture
                glEnable(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, Materials[Objects[i].MatFaces[j].MatIndex].tex);
              } else {
                glColor3f(Materials[Objects[i].MatFaces[j].MatIndex].color.r,
                          Materials[Objects[i].MatFaces[j].MatIndex].color.g,
                          Materials[Objects[i].MatFaces[j].MatIndex].color.b);
              }

              glPushMatrix();

              // Move the model
              glTranslatef(Objects[i].pos.x, Objects[i].pos.y, Objects[i].pos.z);

              // Rotate the model
              glRotatef(Objects[i].rot.z, 0.0f, 0.0f, 1.0f);
              glRotatef(Objects[i].rot.y, 0.0f, 1.0f, 0.0f);
              glRotatef(Objects[i].rot.x, 1.0f, 0.0f, 0.0f);

              // Draw the faces using an index to the vertex array
              glDrawElements(GL_TRIANGLES, Objects[i].MatFaces[j].numSubFaces, GL_UNSIGNED_SHORT, Objects[i].MatFaces[j].subFaces);

              glPopMatrix();

              // Disable texturing
              glDisable(GL_TEXTURE_2D);
            }

          // Disable things
          if (Objects[i].textured)
            glDisableClientState(GL_TEXTURE_COORD_ARRAY);
          if (lit)
            glDisableClientState(GL_NORMAL_ARRAY);
          glDisableClientState(GL_VERTEX_ARRAY);

          // Show the normals?
          if (shownormals)
            {
              // Loop through the vertices and normals and draw the normal
              for (int k = 0; k < Objects[i].numVerts * 3; k += 3)
                {
                  // Disbale lighting if the model is lit
                  if (lit)
                    glDisable(GL_LIGHTING);
                  // Draw the normals blue
                  glColor3f(0.0f, 0.0f, 1.0f);

                  // Draw a line between the vertex and the end of the normal
                  glBegin(GL_LINES);
                  glVertex3f(Objects[i].Vertexes[k], Objects[i].Vertexes[k+1], Objects[i].Vertexes[k+2]);
                  glVertex3f(Objects[i].Vertexes[k]+Objects[i].Normals[k], Objects[i].Vertexes[k+1]+Objects[i].Normals[k+1], Objects[i].Vertexes[k+2]+Objects[i].Normals[k+2]);
                  glEnd();

                  // Reset the color to white
                  glColor3f(1.0f, 1.0f, 1.0f);
                  // If the model is lit then renable lighting
                  if (lit)
                    glEnable(GL_LIGHTING);
                }
            }
        }

      glPopMatrix();
      glPopAttrib();
    }
}

void Model3DS::CalculateNormals()
{
  // Let's build some normals
  for (int i = 0; i < numObjects; i++)
    {
      for (int g = 0; g < Objects[i].numVerts; g++)
        {
          // Reduce each vert's normal to unit
          float length;
          Vector unit;

          unit.x = Objects[i].Normals[g*3];
          unit.y = Objects[i].Normals[g*3+1];
          unit.z = Objects[i].Normals[g*3+2];

          length = (float)sqrt((unit.x*unit.x) + (unit.y*unit.y) + (unit.z*unit.z));

          if (length == 0.0f)
            length = 1.0f;

          unit.x /= length;
          unit.y /= length;
          unit.z /= length;

          Objects[i].Normals[g*3]   = unit.x;
          Objects[i].Normals[g*3+1] = unit.y;
          Objects[i].Normals[g*3+2] = unit.z;
        }
    }
}

void Model3DS::MainChunkProcessor(long length, long findex)
{
  ChunkHeader h;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  while (ftell(bin3ds) < (findex + length - 6)) {
    fread(&h.id, sizeof(h.id), 1, bin3ds);
    fread(&h.len, sizeof(h.len), 1, bin3ds);

    switch (h.id) {
      // This is the mesh information like vertices, faces, and materials
    case EDIT3DS:
      EditChunkProcessor(h.len, ftell(bin3ds));
      break;
      // I left this in case anyone gets very ambitious
    case KEYF3DS:
      //KeyFrameChunkProcessor(h.len, ftell(bin3ds));
      break;
    default:
      break;
    }

    fseek(bin3ds, (h.len - 6), SEEK_CUR);
  }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::EditChunkProcessor(long length, long findex)
{
  ChunkHeader h;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  // First count the number of Objects and Materials
  while (ftell(bin3ds) < (findex + length - 6)) {
    fread(&h.id, sizeof(h.id), 1, bin3ds);
    fread(&h.len, sizeof(h.len), 1, bin3ds);

    switch (h.id) {
    case OBJECT:
      numObjects++;
      break;
    case MATERIAL:
      numMaterials++;
      break;
    default:
      break;
    }

    fseek(bin3ds, (h.len - 6), SEEK_CUR);
  }

  // Now load the materials
  if (numMaterials > 0)
    {
      Materials = new Material[numMaterials];

      // Material is set to untextured until we find otherwise
      for (int d = 0; d < numMaterials; d++)
        Materials[d].textured = false;

      fseek(bin3ds, findex, SEEK_SET);

      int i = 0;

      while (ftell(bin3ds) < (findex + length - 6))
        {
          fread(&h.id,sizeof(h.id),1,bin3ds);
          fread(&h.len,sizeof(h.len),1,bin3ds);

          switch (h.id)
            {
            case MATERIAL	:
              MaterialChunkProcessor(h.len, ftell(bin3ds), i);
              i++;
              break;
            default			:
              break;
            }

          fseek(bin3ds, (h.len - 6), SEEK_CUR);
        }
    }

  // Load the Objects (individual meshes in the whole model)
  if (numObjects > 0)
    {
      Objects = new Object[numObjects];

      // Set the textured variable to false until we find a texture
      for (int k = 0; k < numObjects; k++)
        Objects[k].textured = false;

      // Zero the objects position and rotation
      for (int m = 0; m < numObjects; m++)
        {
          Objects[m].pos.x = 0.0f;
          Objects[m].pos.y = 0.0f;
          Objects[m].pos.z = 0.0f;

          Objects[m].rot.x = 0.0f;
          Objects[m].rot.y = 0.0f;
          Objects[m].rot.z = 0.0f;
        }

      // Zero out the number of texture coords
      for (int n = 0; n < numObjects; n++)
        Objects[n].numTexCoords = 0;

      fseek(bin3ds, findex, SEEK_SET);

      int j = 0;

      while (ftell(bin3ds) < (findex + length - 6))
        {
          fread(&h.id,sizeof(h.id),1,bin3ds);
          fread(&h.len,sizeof(h.len),1,bin3ds);

          switch (h.id)
            {
            case OBJECT	:
              ObjectChunkProcessor(h.len, ftell(bin3ds), j);
              j++;
              break;
            default			:
              break;
            }

          fseek(bin3ds, (h.len - 6), SEEK_CUR);
        }
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::MaterialChunkProcessor(long length, long findex, int matindex)
{
  ChunkHeader h;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  while (ftell(bin3ds) < (findex + length - 6))
    {
      fread(&h.id,sizeof(h.id),1,bin3ds);
      fread(&h.len,sizeof(h.len),1,bin3ds);

      switch (h.id)
        {
        case MAT_NAME	:
          // Loads the material's names
          MaterialNameChunkProcessor(h.len, ftell(bin3ds), matindex);
          break;
        case MAT_AMBIENT	:
          //ColorChunkProcessor(h.len, ftell(bin3ds));
          break;
        case MAT_DIFFUSE	:
          DiffuseColorChunkProcessor(h.len, ftell(bin3ds), matindex);
          break;
        case MAT_SPECULAR	:
          //ColorChunkProcessor(h.len, ftell(bin3ds));
        case MAT_TEXMAP	:
          // Finds the names of the textures of the material and loads them
          TextureMapChunkProcessor(h.len, ftell(bin3ds), matindex);
          break;
        default			:
          break;
        }

      fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::MaterialNameChunkProcessor(long length, long findex, int matindex)
{
  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  // Read the material's name
  for (int i = 0; i < 80; i++)
    {
      Materials[matindex].name[i] = fgetc(bin3ds);
      if (Materials[matindex].name[i] == 0)
        {
          Materials[matindex].name[i] = '\0';
          break;
        }
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::DiffuseColorChunkProcessor(long length, long findex, int matindex)
{
  ChunkHeader h;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  while (ftell(bin3ds) < (findex + length - 6))
    {
      fread(&h.id,sizeof(h.id),1,bin3ds);
      fread(&h.len,sizeof(h.len),1,bin3ds);

      // Determine the format of the color and load it
      switch (h.id)
        {
        case COLOR_RGB	:
          // A rgb float color chunk
          FloatColorChunkProcessor(h.len, ftell(bin3ds), matindex);
          break;
        case COLOR_TRU	:
          // A rgb int color chunk
          IntColorChunkProcessor(h.len, ftell(bin3ds), matindex);
          break;
        case COLOR_RGBG	:
          // A rgb gamma corrected float color chunk
          FloatColorChunkProcessor(h.len, ftell(bin3ds), matindex);
          break;
        case COLOR_TRUG	:
          // A rgb gamma corrected int color chunk
          IntColorChunkProcessor(h.len, ftell(bin3ds), matindex);
          break;
        default			:
          break;
        }

      fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::FloatColorChunkProcessor(long length, long findex, int matindex)
{
  float r;
  float g;
  float b;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  fread(&r,sizeof(r),1,bin3ds);
  fread(&g,sizeof(g),1,bin3ds);
  fread(&b,sizeof(b),1,bin3ds);

  Materials[matindex].color.r = (unsigned char)(r*255.0f);
  Materials[matindex].color.g = (unsigned char)(r*255.0f);
  Materials[matindex].color.b = (unsigned char)(r*255.0f);
  Materials[matindex].color.a = 255;

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::IntColorChunkProcessor(long length, long findex, int matindex)
{
  unsigned char r;
  unsigned char g;
  unsigned char b;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  fread(&r,sizeof(r),1,bin3ds);
  fread(&g,sizeof(g),1,bin3ds);
  fread(&b,sizeof(b),1,bin3ds);

  Materials[matindex].color.r = r;
  Materials[matindex].color.g = g;
  Materials[matindex].color.b = b;
  Materials[matindex].color.a = 255;

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::TextureMapChunkProcessor(long length, long findex, int matindex)
{
  ChunkHeader h;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  while (ftell(bin3ds) < (findex + length - 6))
    {
      fread(&h.id,sizeof(h.id),1,bin3ds);
      fread(&h.len,sizeof(h.len),1,bin3ds);

      switch (h.id)
        {
        case MAT_MAPNAME:
          // Read the name of texture in the Diffuse Color map
          MapNameChunkProcessor(h.len, ftell(bin3ds), matindex);
          break;
        default			:
          break;
        }

      fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::MapNameChunkProcessor(long length, long findex, int matindex)
{
  char name[80];

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  // Read the name of the texture
  for (int i = 0; i < 80; i++)
    {
      name[i] = fgetc(bin3ds);
      if (name[i] == 0)
        {
          name[i] = '\0';
          break;
        }
    }

  // Load the name and indicate that the material has a texture
  char fullname[80];
  sprintf(fullname, "%s%s", path, name);
  textureManager.LoadTexture(&Materials[matindex].tex, fullname, BMP_IMAGE, false);
  printf("%d ", Materials[matindex].tex);
  Materials[matindex].textured = true;

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::ObjectChunkProcessor(long length, long findex, int objindex)
{
  ChunkHeader h;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  // Load the object's name
  for (int i = 0; i < 80; i++)
    {
      Objects[objindex].name[i] = fgetc(bin3ds);
      if (Objects[objindex].name[i] == 0)
        {
          Objects[objindex].name[i] = '\0';
          break;
        }
    }

  while (ftell(bin3ds) < (findex + length - 6))
    {
      fread(&h.id,sizeof(h.id),1,bin3ds);
      fread(&h.len,sizeof(h.len),1,bin3ds);

      switch (h.id)
        {
        case TRIG_MESH	:
          // Process the triangles of the object
          TriangularMeshChunkProcessor(h.len, ftell(bin3ds), objindex);
          break;
        default			:
          break;
        }

      fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::TriangularMeshChunkProcessor(long length, long findex, int objindex)
{
  ChunkHeader h;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  while (ftell(bin3ds) < (findex + length - 6))
    {
      fread(&h.id,sizeof(h.id),1,bin3ds);
      fread(&h.len,sizeof(h.len),1,bin3ds);

      switch (h.id)
        {
        case VERT_LIST	:
          // Load the vertices of the onject
          VertexListChunkProcessor(h.len, ftell(bin3ds), objindex);
          break;
        case LOCAL_COORDS	:
          //LocalCoordinatesChunkProcessor(h.len, ftell(bin3ds));
          break;
        case TEX_VERTS	:
          // Load the texture coordinates for the vertices
          TexCoordsChunkProcessor(h.len, ftell(bin3ds), objindex);
          Objects[objindex].textured = true;
          break;
        default			:
          break;
        }

      fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

  // After we have loaded the vertices we can load the faces
  fseek(bin3ds, findex, SEEK_SET);

  while (ftell(bin3ds) < (findex + length - 6))
    {
      fread(&h.id,sizeof(h.id),1,bin3ds);
      fread(&h.len,sizeof(h.len),1,bin3ds);

      switch (h.id)
        {
        case FACE_DESC	:
          // Load the faces of the object
          FacesDescriptionChunkProcessor(h.len, ftell(bin3ds), objindex);
          break;
        default			:
          break;
        }

      fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::VertexListChunkProcessor(long length, long findex, int objindex)
{
  unsigned short numVerts;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  // Read the number of vertices of the object
  fread(&numVerts,sizeof(numVerts),1,bin3ds);

  // Allocate arrays for the vertices and normals
  Objects[objindex].Vertexes = new GLfloat[numVerts * 3];
  Objects[objindex].Normals = new GLfloat[numVerts * 3];

  // Assign the number of vertices for future use
  Objects[objindex].numVerts = numVerts;

  // Zero out the normals array
  for (int j = 0; j < numVerts * 3; j++)
    Objects[objindex].Normals[j] = 0.0f;

  // Read the vertices, switching the y and z coordinates and changing the sign of the z coordinate
  for (int i = 0; i < numVerts * 3; i+=3)
    {
      fread(&Objects[objindex].Vertexes[i],sizeof(GLfloat),1,bin3ds);
      fread(&Objects[objindex].Vertexes[i+2],sizeof(GLfloat),1,bin3ds);
      fread(&Objects[objindex].Vertexes[i+1],sizeof(GLfloat),1,bin3ds);

      // Change the sign of the z coordinate
      Objects[objindex].Vertexes[i+2] = -Objects[objindex].Vertexes[i+2];
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::TexCoordsChunkProcessor(long length, long findex, int objindex)
{
  // The number of texture coordinates
  unsigned short numCoords;

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  // Read the number of coordinates
  fread(&numCoords,sizeof(numCoords),1,bin3ds);

  // Allocate an array to hold the texture coordinates
  Objects[objindex].TexCoords = new GLfloat[numCoords * 2];

  // Set the number of texture coords
  Objects[objindex].numTexCoords = numCoords;

  // Read teh texture coordiantes into the array
  for (int i = 0; i < numCoords * 2; i+=2)
    {
      fread(&Objects[objindex].TexCoords[i],sizeof(GLfloat),1,bin3ds);
      fread(&Objects[objindex].TexCoords[i+1],sizeof(GLfloat),1,bin3ds);
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::FacesDescriptionChunkProcessor(long length, long findex, int objindex)
{
  ChunkHeader h;
  unsigned short numFaces;	// The number of faces in the object
  unsigned short vertA;		// The first vertex of the face
  unsigned short vertB;		// The second vertex of the face
  unsigned short vertC;		// The third vertex of the face
  unsigned short flags;		// The winding order flags
  long subs;					// Holds our place in the file
  int numMatFaces = 0;		// The number of different materials

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  // Read the number of faces
  fread(&numFaces,sizeof(numFaces),1,bin3ds);

  // Allocate an array to hold the faces
  Objects[objindex].Faces = new GLushort[numFaces * 3];
  // Store the number of faces
  Objects[objindex].numFaces = numFaces * 3;

  // Read the faces into the array
  for (int i = 0; i < numFaces * 3; i+=3)
    {
      // Read the vertices of the face
      fread(&vertA,sizeof(vertA),1,bin3ds);
      fread(&vertB,sizeof(vertB),1,bin3ds);
      fread(&vertC,sizeof(vertC),1,bin3ds);
      fread(&flags,sizeof(flags),1,bin3ds);

      // Place them in the array
      Objects[objindex].Faces[i]   = vertA;
      Objects[objindex].Faces[i+1] = vertB;
      Objects[objindex].Faces[i+2] = vertC;

      // Calculate the face's normal
      Vector n;
      Vertex v1;
      Vertex v2;
      Vertex v3;

      v1.x = Objects[objindex].Vertexes[vertA*3];
      v1.y = Objects[objindex].Vertexes[vertA*3+1];
      v1.z = Objects[objindex].Vertexes[vertA*3+2];
      v2.x = Objects[objindex].Vertexes[vertB*3];
      v2.y = Objects[objindex].Vertexes[vertB*3+1];
      v2.z = Objects[objindex].Vertexes[vertB*3+2];
      v3.x = Objects[objindex].Vertexes[vertC*3];
      v3.y = Objects[objindex].Vertexes[vertC*3+1];
      v3.z = Objects[objindex].Vertexes[vertC*3+2];

      // calculate the normal
      float u[3], v[3];

      // V2 - V3;
      u[0] = v2.x - v3.x;
      u[1] = v2.y - v3.y;
      u[2] = v2.z - v3.z;

      // V2 - V1;
      v[0] = v2.x - v1.x;
      v[1] = v2.y - v1.y;
      v[2] = v2.z - v1.z;

      n.x = (u[1]*v[2] - u[2]*v[1]);
      n.y = (u[2]*v[0] - u[0]*v[2]);
      n.z = (u[0]*v[1] - u[1]*v[0]);

      // Add this normal to its verts' normals
      Objects[objindex].Normals[vertA*3]   += n.x;
      Objects[objindex].Normals[vertA*3+1] += n.y;
      Objects[objindex].Normals[vertA*3+2] += n.z;
      Objects[objindex].Normals[vertB*3]   += n.x;
      Objects[objindex].Normals[vertB*3+1] += n.y;
      Objects[objindex].Normals[vertB*3+2] += n.z;
      Objects[objindex].Normals[vertC*3]   += n.x;
      Objects[objindex].Normals[vertC*3+1] += n.y;
      Objects[objindex].Normals[vertC*3+2] += n.z;
    }

  // Store our current file position
  subs = ftell(bin3ds);

  // Check to see how many materials the faces are split into
  while (ftell(bin3ds) < (findex + length - 6))
    {
      fread(&h.id,sizeof(h.id),1,bin3ds);
      fread(&h.len,sizeof(h.len),1,bin3ds);

      switch (h.id)
        {
        case FACE_MAT	:
          //FacesMaterialsListChunkProcessor(h.len, ftell(bin3ds), objindex);
          numMatFaces++;
          break;
        default			:
          break;
        }

      fseek(bin3ds, (h.len - 6), SEEK_CUR);
    }

  // Split the faces up according to their materials
  if (numMatFaces > 0)
    {
      // Allocate an array to hold the lists of faces divided by material
      Objects[objindex].MatFaces = new MaterialFaces[numMatFaces];
      // Store the number of material faces
      Objects[objindex].numMatFaces = numMatFaces;

      fseek(bin3ds, subs, SEEK_SET);

      int j = 0;

      // Split the faces up
      while (ftell(bin3ds) < (findex + length - 6))
        {
          fread(&h.id,sizeof(h.id),1,bin3ds);
          fread(&h.len,sizeof(h.len),1,bin3ds);

          switch (h.id)
            {
            case FACE_MAT	:
              // Process the faces and split them up
              FacesMaterialsListChunkProcessor(h.len, ftell(bin3ds), objindex, j);
              j++;
              break;
            default			:
              break;
            }

          fseek(bin3ds, (h.len - 6), SEEK_CUR);
        }
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}

void Model3DS::FacesMaterialsListChunkProcessor(long length, long findex, int objindex, int subfacesindex)
{
  char name[80];				// The material's name
  unsigned short numEntries;	// The number of faces associated with this material
  unsigned short Face;		// Holds the faces as they are read
  int material;				// An index to the Materials array for this material

  // move the file pointer to the beginning of the main
  // chunk's data findex + the size of the header
  fseek(bin3ds, findex, SEEK_SET);

  // Read the material's name
  for (int i = 0; i < 80; i++)
    {
      name[i] = fgetc(bin3ds);
      if (name[i] == 0)
        {
          name[i] = '\0';
          break;
        }
    }

  // Faind the material's index in the Materials array
  for (material = 0; material < numMaterials; material++)
    {
      if (strcmp(name, Materials[material].name) == 0)
        break;
    }

  // Store this value for later so that we can find the material
  Objects[objindex].MatFaces[subfacesindex].MatIndex = material;

  // Read the number of faces associated with this material
  fread(&numEntries,sizeof(numEntries),1,bin3ds);

  // Allocate an array to hold the list of faces associated with this material
  Objects[objindex].MatFaces[subfacesindex].subFaces = new GLushort[numEntries * 3];
  // Store this number for later use
  Objects[objindex].MatFaces[subfacesindex].numSubFaces = numEntries * 3;

  // Read the faces into the array
  for (int i = 0; i < numEntries * 3; i+=3)
    {
      // read the face
      fread(&Face,sizeof(Face),1,bin3ds);
      // Add the face's vertices to the list
      Objects[objindex].MatFaces[subfacesindex].subFaces[i] = Objects[objindex].Faces[Face * 3];
      Objects[objindex].MatFaces[subfacesindex].subFaces[i+1] = Objects[objindex].Faces[Face * 3 + 1];
      Objects[objindex].MatFaces[subfacesindex].subFaces[i+2] = Objects[objindex].Faces[Face * 3 + 2];
    }

  // move the file pointer back to where we got it so
  // that the ProcessChunk() which we interrupted will read
  // from the right place
  fseek(bin3ds, findex, SEEK_SET);
}
