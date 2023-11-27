#include "PQP.h"
#include <filesystem>

#ifndef PQP_LOAD_H
#define PQP_LOAD_H

namespace pqploader
{
    void load(const std::string filename);
    bool tiny_OBJ_to_PQP_model(tinyobj::ObjReader &reader, PQP_Model *target_model);
    void read_file(const std::string filename, tinyobj::ObjReader &reader);
}

#endif
