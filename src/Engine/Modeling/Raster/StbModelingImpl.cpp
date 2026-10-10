// Unica unidade com as implementacoes do stb usadas pela modelagem: o escritor
// de PNG e a rasterizacao de fontes TrueType (os rotulos das imagens). O
// ImGui tem a propria copia do stb_truetype, estatica, sem conflito.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"
