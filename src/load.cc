#include <iostream>
#include "tiny_obj_loader.h"
#include "PQP.h"
#include <filesystem>

namespace pqploader
{
    void read_file(const std::string filename, tinyobj::ObjReader &reader)
    {
        tinyobj::ObjReaderConfig reader_config;

        if (!reader.ParseFromFile(filename, reader_config))
        {
            if (!reader.Error().empty())
            {
                std::cerr << "[ERROR] TinyObjReader: " << reader.Error();
            }
            exit(1);
        }

        if (!reader.Warning().empty())
        {
            std::cout << "[WARN] TinyObjReader: " << reader.Warning();
        }
    }

    void load(const std::string filename)
    {
        tinyobj::ObjReaderConfig reader_config;

        // Extract directory from the filename
        std::filesystem::path filePath(filename);
        std::string directory = filePath.parent_path().string();

        reader_config.mtl_search_path = directory;
        reader_config.mtl_search_path = "";
        // reader_config.mtl_search_path = "./"; // Path to material files

        tinyobj::ObjReader reader;

        if (!reader.ParseFromFile(filename, reader_config))
        {
            if (!reader.Error().empty())
            {
                std::cerr << "[ERROR] TinyObjReader: " << reader.Error();
            }
            exit(1);
        }

        if (!reader.Warning().empty())
        {
            std::cout << "[WARN] TinyObjReader: " << reader.Warning();
        }

        auto &attrib = reader.GetAttrib();
        auto &shapes = reader.GetShapes();
        auto &materials = reader.GetMaterials();

        // Loop over shapes
        std::cout << "[INFO] TinyObjReader: number of shapes: " << shapes.size() << std::endl;

        for (size_t s = 0; s < shapes.size(); s++)
        {
            // Loop over faces(polygon)
            size_t index_offset = 0;
            for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
            {
                size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

                // Loop over vertices in the face.
                for (size_t v = 0; v < fv; v++)
                {
                    // access to vertex
                    tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                    tinyobj::real_t vx = attrib.vertices[3 * size_t(idx.vertex_index) + 0];
                    tinyobj::real_t vy = attrib.vertices[3 * size_t(idx.vertex_index) + 1];
                    tinyobj::real_t vz = attrib.vertices[3 * size_t(idx.vertex_index) + 2];

                    // Check if `normal_index` is zero or positive. negative = no normal data
                    if (idx.normal_index >= 0)
                    {
                        tinyobj::real_t nx = attrib.normals[3 * size_t(idx.normal_index) + 0];
                        tinyobj::real_t ny = attrib.normals[3 * size_t(idx.normal_index) + 1];
                        tinyobj::real_t nz = attrib.normals[3 * size_t(idx.normal_index) + 2];
                    }

                    // Check if `texcoord_index` is zero or positive. negative = no texcoord data
                    if (idx.texcoord_index >= 0)
                    {
                        tinyobj::real_t tx = attrib.texcoords[2 * size_t(idx.texcoord_index) + 0];
                        tinyobj::real_t ty = attrib.texcoords[2 * size_t(idx.texcoord_index) + 1];
                    }

                    // Optional: vertex colors
                    // tinyobj::real_t red   = attrib.colors[3*size_t(idx.vertex_index)+0];
                    // tinyobj::real_t green = attrib.colors[3*size_t(idx.vertex_index)+1];
                    // tinyobj::real_t blue  = attrib.colors[3*size_t(idx.vertex_index)+2];
                }
                index_offset += fv;

                // per-face material
                shapes[s].mesh.material_ids[f];
            }
        }
    }

    bool tiny_OBJ_to_PQP_model(tinyobj::ObjReader &reader, PQP_Model *target_model)
    {
        if (!reader.Warning().empty())
        {
            std::cout << "[WARN] TinyObjReader: " << reader.Warning() << std::endl;
        }

        auto &attrib = reader.GetAttrib();
        auto &shapes = reader.GetShapes();

        // assume only one shape per model
        // if (shapes.size() > 1)
        // {
        //     throw std::runtime_error("[ERROR] TinyObjReader: OBJ must have only one shape in function `tiny_OBJ_to_PQP_model`, currently has: " + std::to_string(shapes.size()));
        // }
        auto &materials = reader.GetMaterials();

        // Loop over shapes
        std::cout << "[INFO] TinyObjReader: number of shapes: " << shapes.size() << std::endl;

        target_model->BeginModel();
        for (int s = 0; s < shapes.size(); s++)
        {
            tinyobj::shape_t shape = shapes[s];
            // Loop over faces(polygon)
            size_t index_offset = 0;

            std::cout << "[INFO] TinyObjReader: number of faces: " << shape.mesh.num_face_vertices.size() << std::endl;
            for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++)
            {
                size_t fv = size_t(shape.mesh.num_face_vertices[f]);

                if (fv != 3)
                {
                    throw std::runtime_error("[ERROR] Mesh must be triangulated.");
                }

                PQP_REAL ps[3][3];
                // Loop over vertices in the face.
                for (size_t v = 0; v < fv; v++)
                {
                    PQP_REAL *psv = ps[v];
                    // access to vertex
                    tinyobj::index_t idx = shape.mesh.indices[index_offset + v];
                    int vertex_index = size_t(idx.vertex_index);

                    for (size_t i = 0; i < 3; ++i)
                    {
                        tinyobj::real_t vi = attrib.vertices[3 * vertex_index + i];
                        // typedef PQP_REAL double
                        psv[i] = (PQP_REAL)vi;
                    }
                }
                target_model->AddTri(ps[0], ps[1], ps[2], f);
                // always 3 in this case because all the faces are triangles
                index_offset += fv;
            }
        }
        target_model->EndModel();
        return true;
    }
}