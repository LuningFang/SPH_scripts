// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Luning
// Demo for converting obj terrain mesh to SPH marker 
// use omp for speedup
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include <omp.h>


#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/ChVisualizationFsi.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_vehicle/terrain/SPHTerrain.h"

#include <iostream>
#include <fstream>

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::geometry;
using namespace chrono::viper;
using namespace chrono::vehicle;

// output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Terrain_larger/";

// if true, save as Wavefront OBJ; if false, save as VTK
bool save_obj = false;  

// Physical properties of terrain particles
double iniSpacing = 0.02;
double kernelLength = iniSpacing;
double density = 1700.0;

// Dimension of the space domain
double bxDim = 20.0;
double byDim = 20.0;
double bzDim = 0.2;

// Define L shaped terrain for the rover
double rec1_xstart = -0.25 - 0.5;
double rec2_xstart = 2.15 - 0.5;
double rec2_xend = 4.25 - 0.5;

double ystart = -4.35;
double rec1_yend = -2.0;
// double rec2_yend = 9.0;
double rec2_yend = -1; // testing purpose

// Define boxed particle domain
ChVector<double> sph_markers_min(-2, -6, -1.0);
ChVector<double> sph_markers_max( 6, 10,  0.5);


// ChVector<double> sph_markers_min(-2, -6, -0.5);
// ChVector<double> sph_markers_max( 0, 10, 0);


// Rover initial location
ChVector<> init_loc(rec1_xstart + 0.8f, -3.0, bzDim + 0.25f);

// Simulation time and stepsize
double dT = 2.5e-4;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;


std::vector<std::vector<double>> importCSV(const std::string& filename) {
    std::vector<std::vector<double>> data;

    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return data; // Return empty vector if file opening fails
    }

    std::string line;
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) {
            try {
                double value = std::stod(cell);
                row.push_back(value);
            } catch (const std::exception& e) {
                std::cerr << "Failed to convert cell value to double: " << cell << std::endl;
                row.clear(); // Clear the row if conversion fails
                break;
            }
        }
        if (!row.empty()) {
            data.push_back(row);
        }
    }

    return data;
}

int main(int argc, char* argv[]) {
    omp_set_num_threads(16);
    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/rover"))) {
        std::cerr << "Error creating directory " << out_dir + "/rover" << std::endl;
        return 1;
    }

    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    // Create a ground body for the BCE particles
    std::shared_ptr<ChBody> m_ground = std::shared_ptr<ChBody>(sysMBS.NewBody());
    m_ground->SetBodyFixed(true);
    sysMBS.AddBody(m_ground);

    ChVector<> gravity = ChVector<>(0, 0, -9.81);
    sysMBS.Set_G_acc(gravity);
    sysFSI.Set_G_acc(gravity);

    // Read JSON file with simulation parameters
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Viper_granular_NSC.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc != 1) {
        std::cout << "usage: ./demo_ROBOT_Viper_SPH <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);

    // Set the initial particle spacing
    sysFSI.SetInitialSpacing(iniSpacing);

    // Set the SPH kernel length
    sysFSI.SetKernelLength(kernelLength);

    // Set the terrain density
    sysFSI.SetDensity(density);

    // Set the simulation stepsize
    sysFSI.SetStepSize(dT);

    // Set the simulation domain size
    sysFSI.SetContainerDim(ChVector<>(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetDiscreType(false, false);

    // Set wall boundary condition
    sysFSI.SetWallBC(BceVersion::ADAMI);

    // Setup the solver based on the input value of the prameters
    sysFSI.SetSPHMethod(FluidDynamics::WCSPH);

    // Set the periodic boundary condition
    // ChVector<> cMin(-bxDim / 2 * 2, -byDim / 2 - 0.5 * iniSpacing, -bzDim * 10);
    // ChVector<> cMax(bxDim / 2 * 2, byDim / 2  + 0.5 * iniSpacing, bzDim * 20);
    ChVector<> cMin(-15.0f, -15.0f, -3.0f);
    ChVector<> cMax(15.0f, 15.0f, 3.0f);
    sysFSI.SetBoundaries(cMin, cMax);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Separating the layers, one for the terrain, one for BCE, one for visualization
    // std::cout << "Start reading point cloud files" << std::endl;
    // auto read_pts = importCSV("/home/luning/Source/sensor/build/data/fsi/terrain/sph_terrain_18.csv");
    // auto read_pts_top = importCSV("/home/luning/Source/sensor/build/data/fsi/terrain/top_layer.csv");
    // auto read_pts_bot = importCSV("/home/luning/Source/sensor/build/data/fsi/terrain/dense_blayer_3.csv");
    // std::cout << "Finished reading point cloud files" << std::endl;

    // Size of the terrain file
    // int size_terrain = (int)read_pts.size();
    // int size_terrain_top = (int)read_pts_top.size();
    // int size_terrain_bot = (int)read_pts_bot.size();
    // std::cout << "Size of the top layer terrain file: " << size_terrain_top << std::endl;
    // std::cout << "Size of the terrain file: " << size_terrain << std::endl;
    // std::cout << "Size of the bottom layer terrain file: " << size_terrain_bot << std::endl;

    // Add SPH particles from the sampler points to the FSI system
    auto gz = std::abs(gravity.z());

    // Count Num. SPH particles
    int sph_count = 0;

    // Define BCE point vector
    std::vector<ChVector<>> bce_points;

    // Defining how many extra BCE particles to add
    double bce_const = 0.05;

    double min_z = DBL_MAX;

    // load lunar terrain mesh
    auto terrain_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    std::string terrain_mesh_filename = GetChronoDataFile("fsi/terrain/smaller_terrain.obj");
    double scale_ratio = 1.0;
    std::cout << "Loading terrain mesh from : " << terrain_mesh_filename << std::endl;
    terrain_mesh->LoadWavefrontMesh(GetChronoDataFile(terrain_mesh_filename), false, true);
    terrain_mesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    terrain_mesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight


    // ray direction pointing up
    ChVector<double> ray_dir(0,0,1);
    const double EPSI = 1e-6;

    std::cout << "number of triangles loaded: " << terrain_mesh->m_face_v_indices.size();

    std::string filename = "particle_file_layer_test.csv";
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cout << "Error opening file: " << filename << std::endl;
    }

    // Write header row
    file << "X,Y,Z" << std::endl;

    int numX      = int((rec2_xend   - rec1_xstart) / iniSpacing);
    int numX_part = int((rec2_xstart - rec1_xstart) / iniSpacing);
    int numY_part = int((rec1_yend - ystart) / iniSpacing);
    int numY_all  = int((rec2_yend - ystart) / iniSpacing);
    int numY;
    double ray_origin_x, ray_origin_y, ray_origin_z;
    // loop through every point in sph domain
    for (unsigned int ix = 0; ix < numX; ix ++){
        double x = rec1_xstart + iniSpacing * ix;        
        ray_origin_x = x + 1e-9;
        std::cout << "particles x = " << x << std::endl;

        numY = (ix >= numX_part) ? numY_all : numY_part;

        #pragma omp parallel for
        for (unsigned int iy = 0; iy < numY; iy++) {
            double y = ystart + iniSpacing * iy;
            ray_origin_y = y + 1e-9;

            for (double z = sph_markers_max.z(); z > sph_markers_min.z(); z -= iniSpacing) {
                ray_origin_z = z + 1e-9;
                bool surfacePointFound = false;
                for (unsigned int i = 0; i < terrain_mesh->m_face_v_indices.size(); ++i) {
                    auto& t_face = terrain_mesh->m_face_v_indices[i];
                    auto& v1 = terrain_mesh->m_vertices[t_face.x()];
                    auto& v2 = terrain_mesh->m_vertices[t_face.y()];
                    auto& v3 = terrain_mesh->m_vertices[t_face.z()];

                    // no need to proceed if the point is outside the bounding rectangle of the triangle
                    double x_min = ChMin(ChMin(v1.x(), v2.x()), v3.x());
                    double x_max = ChMax(ChMax(v1.x(), v2.x()), v3.x());
                    double y_min = ChMin(ChMin(v1.y(), v2.y()), v3.y());
                    double y_max = ChMax(ChMax(v1.y(), v2.y()), v3.y());
                    ChVector<double> ray_origin(ray_origin_x, ray_origin_y, ray_origin_z);
                    if (ray_origin.x() < x_min || ray_origin.x() > x_max || 
                        ray_origin.y() < y_min || ray_origin.y() > y_max
                        ){
                            continue;
                        }

                    // Find vectors for two edges sharing V1
                    auto edge1 = v2 - v1;
                    auto edge2 = v3 - v1;

                    // Begin calculating determinant - also used to calculate uu parameter
                    auto pvec = Vcross(ray_dir, edge2);
                    // if determinant is near zero, ray is parallel to plane of triangle
                    double det = Vdot(edge1, pvec);
                    // NOT CULLING
                    if (det > -EPSI && det < EPSI) {
                        continue;
                    }
                    double inv_det = 1.0 / det;

                    // calculate distance from V1 to ray origin
                    auto tvec = ray_origin - v1;

                    /// Calculate uu parameter and test bound
                    double uu = Vdot(tvec, pvec) * inv_det;
                    // The intersection lies outside of the triangle
                    if (uu < 0.0 || uu > 1.0) {
                        continue;
                    }

                    // Prepare to test vv parameter
                    auto qvec = Vcross(tvec, edge1);

                    // Calculate vv parameter and test bound
                    double vv = Vdot(ray_dir, qvec) * inv_det;
                    // The intersection lies outside of the triangle
                    if (vv < 0.0 || ((uu + vv) > 1.0)) {
                        continue;
                    }

                    double tt = Vdot(edge2, qvec) * inv_det;
                    if (tt > EPSI) {  /// ray intersection
                        double pre_ini = sysFSI.GetDensity() * gz * (z + bzDim);
                        //  sysFSI.AddSPHParticle(ChVector<>(x, y, z), sysFSI.GetDensity(), 0, sysFSI.GetViscosity(),
                        //     ChVector<>(0),         // initial velocity
                        //     ChVector<>(-pre_ini),  // tauxxyyzz
                        //     ChVector<>(0)          // tauxyxzyz
                        //                 );
                        // std::cout << "add particle " << point << std::endl;
                        #pragma omp critical                        
                        file << x << "," << y << "," << z << std::endl;
                        surfacePointFound = true;
                        break;
                    }

                }

                if (surfacePointFound == true) {
                    break;
                }
            
            }
        }
    }


    file.close();

    ChVector<double> container_pos = (sph_markers_min + sph_markers_max)/2.0f;
    ChVector<double> container_dim = (sph_markers_max - sph_markers_min) + 2 * iniSpacing *  ChVector<double>(1, 1, 1);

    container_pos.z() = sph_markers_min.z();
    sysFSI.AddContainerBCE(m_ground, ChFrame<>(container_pos, QUNIT), container_dim, ChVector<int>(2, 2, -1));





    // Create the wheel's BCE particles
    // auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    // trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    // trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    // trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    // Add BCE particles and mesh of wheels to the system
    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Create a run-tme visualizer
    ChVisualizationFsi fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Viper on SPH terrain");
        fsi_vis.SetSize(1920, 1080);
        fsi_vis.UpdateCamera(init_loc + ChVector<>(7.5f, 13.0f, 1.9f), init_loc);
        fsi_vis.SetCameraMoveScale(0.2f);
        fsi_vis.EnableBoundaryMarkers(false);
        fsi_vis.EnableRigidBodyMarkers(false);
        fsi_vis.AttachSystem(&sysMBS);
        fsi_vis.Initialize();
    }

    // Start the simulation
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    double time = 0.0;
    int current_step = 0;
    double total_time = 5.0;

    auto body = sysMBS.Get_bodylist()[1];

    double steering = 0;
    double mid = (rec2_xstart + rec2_xend) / 2;
    double max_steering = CH_C_PI / 3;

    ChTimer<> timer;
    int output_steps = 40;
    while (time < total_time) {
        std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << "  RTF: " << timer() / time << std::endl;
        


        // Render system
        if (render && current_step % render_steps == 0) {
            if (!fsi_vis.Render())
                break;
        }

        timer.start();
        if (current_step % output_steps == 0) {
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            // sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
        }
        sysFSI.DoStepDynamics_FSI();


        timer.stop();

        time += dT;
        current_step++;
    }

    return 0;
}